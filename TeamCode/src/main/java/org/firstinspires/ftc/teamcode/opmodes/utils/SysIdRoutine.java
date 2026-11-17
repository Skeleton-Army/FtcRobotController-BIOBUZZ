package org.firstinspires.ftc.teamcode.opmodes.utils;

import android.annotation.SuppressLint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.skeletonarmy.marrow.LynxUtil;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;

/**
 * Flywheel system identification: low / medium / high voltage spin-up tests.
 *
 * Workflow (one OpMode run = one test = one CSV file):
 *   1. INIT: use the Prompter (D-PAD + A) to pick the level and the run number, confirm on the summary.
 *   2. PLAY, then press A to start. The SysIdCommand:
 *        - records ~PRE_REST_S at rest (0 V),
 *        - applies the selected voltage and records while the flywheel accelerates,
 *        - detects when the speed has settled, records HOLD_S more seconds,
 *        - stops the motor and saves the CSV automatically.
 *      Press X at any time to abort (motor stops, nothing is saved).
 *   3. Stop the OpMode, re-INIT, pick the next level / run. Do LOW, MEDIUM, HIGH, then all three again.
 *
 * CSV: time_s, voltage_V, battery_voltage_V, position_rad, velocity_rad_s
 * (voltage_V = motor power * battery voltage = the model input u)
 * File: /sdcard/FIRST/sysid/sysid_<level>_run<n>_<timestamp>.csv
 */
@Utility
public class SysIdRoutine extends CommandOpMode {

    // ======================= CONFIG =======================
    private static final String MOTOR_NAME = "flywheel";
    private static final boolean MOTOR_REVERSED = true; // flip if the wheel spins the wrong way
    private static final boolean ENCODER_REVERSED = false; // flip if positive voltage gives negative velocity

    private static final double PRE_REST_S = 0.25; // recorded at rest before voltage is applied
    private static final double HOLD_S = 2.0; // extra recording after the speed has settled
    private static final double MAX_SPINUP_S = 10.0; // give up waiting for settling after this long

    private static final double SETTLE_WINDOW_S = 0.5; // speed must stay flat for this long...
    private static final double SETTLE_TOL = 0.02; // ...within this fraction of itself
    private static final double MIN_SETTLED_VEL = 1.0; // rad/s, so "at rest" never counts as settled
    private static final double VEL_FILTER_ALPHA = 0.2; // low-pass on velocity, used for settle detection only

    private static final double VOLTAGE_REFRESH_S = 0.1; // how often to re-read battery voltage (slow call)
    private static final String SAVE_DIR = "/sdcard/FIRST/sysid/";

    /** Test levels. Tune the voltages for your flywheel. */
    private enum Level {
        LOW("low", 4.0),
        MEDIUM("medium", 6.0),
        HIGH("high", 8.0);

        final String id;
        final double volts;

        Level(String id, double volts) {
            this.id = id;
            this.volts = volts;
        }

        @Override
        public String toString() {
            return name() + " (" + volts + " V)";
        }
    }
    // ======================================================

    private enum Phase {
        REST,
        SPINUP,
        HOLD,
        DONE
    }

    private FlywheelSubsystem flywheel;
    private GamepadEx driver;
    private Prompter prompter;
    private SysIdCommand sysIdCommand;

    private boolean promptsDone = false;
    private boolean testStarted = false;
    private Level level;
    private int runNumber;

    private String status = "";

    // ======================= OPMODE =======================

    @Override
    public void initialize() {
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        driver = new GamepadEx(gamepad1);
        flywheel = new FlywheelSubsystem(hardwareMap);
        register(flywheel);

        // ---------- test selection (Prompter) ----------
        // To add more options, just add enum values / prompts here.
        prompter = new Prompter(this);
        prompter.prompt("level", new OptionPrompt<>("Select Test Level", Level.LOW, Level.MEDIUM, Level.HIGH))
                .label("Level")
                .prompt("run", new OptionPrompt<>("Select Run Number", 1, 2, 3))
                .label("Run #")
                .showSummary()
                .onComplete(() -> {
                    level = prompter.get("level");
                    runNumber = prompter.get("run");
                    promptsDone = true;
                });

        // ---------- bindings ----------
        sysIdCommand = new SysIdCommand();

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
            // Guard so that pressing A inside the Prompter during INIT can never start the test.
            if (!testStarted && promptsDone && isStarted()) {
                testStarted = true;
                schedule(sysIdCommand);
            }
        }));

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
            if (CommandScheduler.getInstance().isScheduled(sysIdCommand)) {
                CommandScheduler.getInstance().cancel(sysIdCommand);
            }
        }));
    }

    /** Runs repeatedly during INIT (like init_loop): drives the Prompter, then shows the selection. */
    @Override
    public void initialize_loop() {
        if (!promptsDone) {
            prompter.run();
        } else {
            telemetry.addData("Selected", "%s, run %d", level, runNumber);
            telemetry.addLine("Press PLAY, then A to start the test");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        LynxUtil.clearBulkCache(hardwareMap);
        driver.readButtons();
        super.run(); // CommandScheduler.run()

        if (!promptsDone) {
            telemetry.addLine("Prompts were not completed - stop and re-init");
        } else if (!testStarted) {
            telemetry.addData("Ready", "%s, run %d", level, runNumber);
            telemetry.addLine("Press A to start (X aborts)");
        } else if (CommandScheduler.getInstance().isScheduled(sysIdCommand)) {
            telemetry.addData("Phase", sysIdCommand.phase);
            telemetry.addData("Time", sysIdCommand.t);
            telemetry.addData("Battery V", sysIdCommand.battery);
            telemetry.addData("Velocity (rad/s)", sysIdCommand.vel);
            telemetry.addLine("X to abort");
        } else {
            telemetry.addData("Status", status);
            telemetry.addLine("Stop the OpMode and re-init for the next test");
        }
        telemetry.update();
    }

    // ======================= SUBSYSTEM =======================

    private static class FlywheelSubsystem extends SubsystemBase {
        private final MotorEx motor;
        private final VoltageSensor voltageSensor;
        private final double encSign = ENCODER_REVERSED ? -1 : 1;

        FlywheelSubsystem(HardwareMap hardwareMap) {
            motor = new MotorEx(hardwareMap, MOTOR_NAME, Motor.GoBILDA.BARE);
            motor.setInverted(MOTOR_REVERSED);
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        void setPower(double power) {
            motor.set(power);
        }

        void stop() {
            motor.set(0);
        }

        /** Slow call: throttle it. */
        double getBatteryVoltage() {
            return voltageSensor.getVoltage();
        }

        double getPositionRad() {
            return encSign * ticksToRad(motor.getCurrentPosition());
        }

        double getVelocityRadPerSec() {
            return encSign * ticksToRad(motor.getVelocity());
        }

        private double ticksToRad(double ticks) {
            return ticks * (2 * Math.PI / motor.getCPR());
        }
    }

    // ======================= COMMAND =======================

    /** Runs one full test. Finishes when done; interrupted (X / OpMode stop) means nothing is saved. */
    private class SysIdCommand extends CommandBase {
        private final List<double[]> records = new ArrayList<>(2000);

        // exposed for telemetry
        Phase phase = Phase.REST;
        double t, battery, vel;

        private long startNanos;
        private double lastVoltageReadT;
        private double applyT, settledT;
        private double filtVel;
        private double winStart, winMin, winMax;
        private boolean timedOut;

        SysIdCommand() {
            addRequirements(flywheel);
        }

        @Override
        public void initialize() {
            records.clear();
            phase = Phase.REST;
            timedOut = false;
            applyT = settledT = 0;
            filtVel = 0;
            winStart = winMin = winMax = 0;
            lastVoltageReadT = 0;
            battery = flywheel.getBatteryVoltage();
            startNanos = System.nanoTime();
        }

        @Override
        public void execute() {
            t = (System.nanoTime() - startNanos) * 1e-9;

            if (t - lastVoltageReadT >= VOLTAGE_REFRESH_S) {
                battery = flywheel.getBatteryVoltage();
                lastVoltageReadT = t;
            }

            // ---- phase transitions ----
            if (phase == Phase.REST && t >= PRE_REST_S) {
                phase = Phase.SPINUP;
                applyT = t;
                filtVel = Double.NaN; // initialised on first spin-up sample
            }

            double power = (phase == Phase.REST) ? 0 : Math.max(-1, Math.min(1, level.volts / battery));
            flywheel.setPower(power);

            double pos = flywheel.getPositionRad();
            vel = flywheel.getVelocityRadPerSec();

            records.add(new double[] {t, power * battery, battery, pos, vel});

            // ---- settle detection ----
            if (phase == Phase.SPINUP) {
                if (Double.isNaN(filtVel)) {
                    filtVel = vel;
                    winStart = t;
                    winMin = winMax = filtVel;
                } else {
                    filtVel += VEL_FILTER_ALPHA * (vel - filtVel);
                    winMin = Math.min(winMin, filtVel);
                    winMax = Math.max(winMax, filtVel);
                }

                if (t - winStart >= SETTLE_WINDOW_S) {
                    double span = winMax - winMin;
                    if (Math.abs(filtVel) > MIN_SETTLED_VEL && span <= SETTLE_TOL * Math.abs(filtVel)) {
                        phase = Phase.HOLD;
                        settledT = t;
                    } else {
                        winStart = t;
                        winMin = winMax = filtVel;
                    }
                }

                if (phase == Phase.SPINUP && t - applyT > MAX_SPINUP_S) {
                    timedOut = true;
                    phase = Phase.DONE;
                }
            } else if (phase == Phase.HOLD && t - settledT >= HOLD_S) {
                phase = Phase.DONE;
            }
        }

        @Override
        public boolean isFinished() {
            return phase == Phase.DONE;
        }

        @Override
        public void end(boolean interrupted) {
            flywheel.stop();
            if (interrupted) {
                status = "Aborted - nothing saved";
            } else {
                status = timedOut ? "Timed out before settling (saving anyway)" : "Test complete";
                saveData(records);
            }
        }
    }

    // ======================= SAVING =======================

    @SuppressLint("SdCardPath")
    private void saveData(List<double[]> records) {
        File dir = new File(SAVE_DIR);
        dir.mkdirs();
        File file = new File(dir, "sysid_" + level.id + "_run" + runNumber + "_" + System.currentTimeMillis() + ".csv");

        CSVFormat csvFormat = CSVFormat.DEFAULT
                .builder()
                .setHeader("time_s", "voltage_V", "battery_voltage_V", "position_rad", "velocity_rad_s")
                .setAutoFlush(false)
                .get();

        try (CSVPrinter printer = new CSVPrinter(new FileWriter(file), csvFormat)) {
            for (double[] r : records) {
                printer.printRecord(r[0], r[1], r[2], r[3], r[4]);
            }
            printer.flush();
            status += " | Saved " + file.getName() + " (" + records.size() + " samples)";
        } catch (IOException e) {
            status = "SAVE FAILED: " + e.getMessage();
        }
    }
}
