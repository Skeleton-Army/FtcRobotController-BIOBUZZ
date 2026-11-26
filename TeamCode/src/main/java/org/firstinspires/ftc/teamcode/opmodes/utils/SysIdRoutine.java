package org.firstinspires.ftc.teamcode.opmodes.utils;

import android.annotation.SuppressLint;
import android.util.Log;
import android.util.Pair;

import androidx.annotation.NonNull;

import com.pedropathing.localization.FusionLocalizer;
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
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;

import okhttp3.HttpUrl;
import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

/**
 * Flywheel system identification (position-based step-response tests for kS / kV / kA).
 *
 * Mechanism: the assembled flywheel + transmission, forward direction only, no balls, open loop
 * (RawPower = RUN_WITHOUT_ENCODER, no velocity controller).
 *
 * Runs (one OpMode run = one test = one CSV file):
 *   A1, B1, C1, A2, B2, C2 - fitting runs (~4 / 6 / 8 V, each twice)
 *   D                      - validation run (~7 V), excluded from the fit
 *   DOWN                   - deceleration validation run (~8 V, settle, step down to ~4 V, settle), excluded from the fit
 *
 * Workflow:
 *   1. INIT: pick the run with the Prompter (D-PAD + A). Check that spinning the flywheel forward by hand
 *      increases the position shown, otherwise flip ENCODER_REVERSED.
 *   2. PLAY, make sure the flywheel is stopped, then press A. The SysIdCommand:
 *        - measures the battery voltage and fixes the power p = V_target / V_battery_start for the whole run
 *          (the run is refused if p would exceed 1),
 *        - if the run has a start voltage: spins up to it and waits until settled, WITHOUT recording,
 *          then re-zeroes the clock (time_s = 0 is the first recorded moment),
 *        - records PRE_REST_S at rest (or at the start voltage), then steps to p and logs the command-change time,
 *        - judges settling from the average speed over ~0.5 s windows of the position,
 *        - records HOLD_S more after settling, then stops the motor and saves the CSV.
 *      Press X at any time to abort (motor stops, nothing is saved).
 *   3. Stop the OpMode, let the flywheel stop (and the motor cool), check for loose parts / rubbing, re-INIT, pick the next run.
 *
 * All samples are buffered in memory and written only after the motor has stopped.
 *
 * CSV columns:
 *   time_s                 - monotonic time of the position sample
 *   position_rad           - continuous position (motor shaft or flywheel shaft, see SHAFT_REVS_PER_MOTOR_REV)
 *   power_command          - power sent to the motor, [-1, 1]
 *   battery_voltage_V      - most recent supply voltage reading
 *   battery_time_s         - time at which that voltage reading was taken (voltage is sampled asynchronously)
 *   command_change_time_s  - time at which the current power_command was sent
 *                            (negative if it was sent during an unrecorded start-voltage pre-spin)
 * In MATLAB use V(t) ~= power_command * battery_voltage_V rather than the nominal target.
 * File: /sdcard/FIRST/sysid/sysid_<run>_<timestamp>.csv
 */
@Utility
public class SysIdRoutine extends CommandOpMode {

    // ======================= CONFIG =======================
    private static final String MOTOR_NAME = "flywheel";
    private static final boolean MOTOR_REVERSED = true; // flip if the wheel spins the wrong way
    private static final boolean ENCODER_REVERSED = false; // flip if forward motion gives decreasing position

    // Which shaft does position_rad describe? 1.0 = motor shaft. For the flywheel shaft, use
    // (flywheel revs per motor rev), e.g. 1.5 for a 1:1.5 step-up. The fitted gains correspond to that shaft.
    private static final double SHAFT_REVS_PER_MOTOR_REV = 1.0;

    private static final double VEL_MOTOR_STOPPED_RPM = 50.0;

    private static final double PRE_REST_S = 0.5; // recorded at rest before power is applied
    private static final double HOLD_S = 2.0; // extra recording after the speed has settled (1-2 s)
    private static final double MAX_SPINUP_S = 10.0; // per step: give up waiting for settling after this long

    private static final double SETTLE_WINDOW_S = 0.5; // average speed is computed over windows of this length
    private static final double SETTLE_TOL = 0.02; // successive window speeds must differ by less than this fraction
    private static final int SETTLE_STABLE_COMPARISONS = 2; // ...this many times in a row (~1-1.5 s of no trend)
    private static final double MIN_SETTLED_VEL = 1.0; // rad/s, so "at rest" never counts as settled

    private static final double VOLTAGE_REFRESH_S = 0.1; // how often to re-read battery voltage (slow call)
    private static final double TELEMETRY_PERIOD_S = 0.25; // telemetry is throttled during a run
    private static final String SAVE_DIR = "/sdcard/FIRST/sysid/";
    private static final HttpUrl SERVER = HttpUrl.parse("http://192.168.43.40:3923/?pw=wark");

    /** Test runs. Adjust the voltages if the low level does not move reliably or you need a wider speed range. */
    private enum Run {
        /*
        // ---- fitting runs: step up from rest, fixed voltage, each done twice ----
        FIT_4V_1("fit_4v_1", "fit", true, 4.0),
        FIT_6V_1("fit_6v_1", "fit", true, 6.0),
        FIT_8V_1("fit_8v_1", "fit", true, 8.0),
        FIT_4V_2("fit_4v_2", "fit", true, 4.0),
        FIT_6V_2("fit_6v_2", "fit", true, 6.0),
        FIT_8V_2("fit_8v_2", "fit", true, 8.0),

        // ---- exploratory: step up from rest at higher voltages ----
        TEST_UP_9_5V("test_up_9_5v", "test", true, 9.5),
        TEST_UP_10V("test_up_10v", "test", true, 10.0),
        TEST_UP_7V("test_up_7v", "test", true, 7.0),

        // ---- exploratory: step sequences, no rest baseline ----
        TEST_9V_TO_4V("test_9v_to_4v", "test", false, 9.0, 4.0),
        TEST_SEQ_9_4_12_6_10V("test_seq_9_4_12_6_10v", "test", false, 9.0, 4.0, 12.0, 6.0, 10.0),
        TEST_SEQ_4_10_6V_1("test_seq_4_10_6v_1", "test", false, 4.0, 10.0, 6.0),
        TEST_SEQ_4_10_6V_2("test_seq_4_10_6v_2", "test", false, 4.0, 10.0, 6.0),

         */

        // ---- deceleration: start high, step down ----
        DECEL_OPT_1("decel_opt_9_to_4v", "deceleration opt", 9, true, true, 4),
        DECEL_OPT_2("decel_opt_12_to_8v", "deceleration opt", 12, true, true, 8),
        DECEL_TEST_1("decel_test_10_to_5v", "deceleration test", 10, true, true, 5),
        DECEL_TEST_2("decel_test_12_to_5v", "deceleration test", 12, true, true, 5),
        DECEL_TEST_3("decel_test_11_to_6v", "decel_test", 11, true, true, 6),

        // ---- acceleration: start low, step up ----
        ACCEL_TEST_5_TO_10V("accel_test_5_to_10v", "accel test", 5, true, true, 10),

        // ---- held-out validation ----
        VAL_7V("val_7v", "validation", true, 7.0),
        VAL_8V_TO_4V("val_8v_to_4v", "validation", false, 8.0, 4.0); // no rest baseline: settle at 8 V, then step down to 4 V

        final String id;
        final String role;
        final boolean restFirst;
        final double[] volts; // one entry per step; each step is held until settled
        final boolean fullStop;
        final double startVolts; // > 0: spin up to this voltage first and wait until settled; none of that is recorded

        Run(String id, String role, boolean restFirst, double... volts) {
            this(id, role, restFirst, true, volts);
        }

        Run(String id, String role, boolean restFirst, boolean fullStop, double... volts) {
            this(id, role, 0.0, restFirst, fullStop, volts);
        }

        /**
         * With a start voltage. The steps are passed as an array to keep the overloads unambiguous,
         * e.g. {@code new double[] {9.0, 4.0}}. If restFirst is true, PRE_REST_S is recorded at the start voltage.
         */
        Run(String id, String role, double startVolts, boolean restFirst, boolean fullStop, double... volts) {
            this.id = id;
            this.role = role;
            this.restFirst = restFirst;
            this.fullStop = fullStop;
            this.startVolts = startVolts;
            this.volts = volts;
        }

        @NonNull
        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder(id).append(" (").append(role).append(", ");
            if (startVolts > 0) sb.append("start ").append(startVolts).append(" V, ");
            for (int i = 0; i < volts.length; i++) {
                if (i > 0) sb.append(" -> ");
                sb.append(volts[i]).append(" V");
            }
            return sb.append(")").toString();
        }
    }
    // ======================================================

    private enum Phase {
        PRESPIN, // optional start voltage: spin up and settle, not recorded
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
    private Run selectedRun;

    private String status = "";
    private long lastTelemetryNanos = 0;

    // ======================= OPMODE =======================

    @Override
    public void initialize() {
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        driver = new GamepadEx(gamepad1);
        flywheel = new FlywheelSubsystem(hardwareMap);
        register(flywheel);

        // ---------- test selection (Prompter) ----------
        prompter = new Prompter(this);
        prompter.prompt("run", new OptionPrompt<>("Select Run", Run.values()))
                .label("Run")
                .prompt("upload", new BooleanPrompt("Upload log to HTTP server: " + SERVER.host(), true))
                .label("Upload to server")
                .prompt("remove", new BooleanPrompt("Delete file on bot after test", false))
                .label("Remove file after test")
                .showIf("upload", true)
                .showSummary()
                .onComplete(() -> {
                    selectedRun = prompter.get("run");
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
        LynxUtil.clearBulkCache(hardwareMap);
        if (!promptsDone) {
            prompter.run();
        } else {
            telemetry.addData("Selected", selectedRun);
            telemetry.addData("Position (rad)", flywheel.getPositionRad());
            telemetry.addLine("Spin the flywheel forward by hand: position must increase");
            telemetry.addLine("Press PLAY, then A to start the test");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        LynxUtil.clearBulkCache(hardwareMap);
        driver.readButtons();
        super.run(); // CommandScheduler.run()

        if (testStarted && CommandScheduler.getInstance().isScheduled(sysIdCommand)) {
            // Throttle telemetry so it never slows down acquisition.
            long now = System.nanoTime();
            if (now - lastTelemetryNanos < TELEMETRY_PERIOD_S * 1e9) return;
            lastTelemetryNanos = now;

            telemetry.addData("Run", selectedRun);
            telemetry.addData("Phase", sysIdCommand.phase);
            telemetry.addData("Step", "%d/%d", sysIdCommand.stage + 1, selectedRun.volts.length);
            telemetry.addData("Time (s)", sysIdCommand.t);
            telemetry.addData("Power", sysIdCommand.power);
            telemetry.addData("Battery V", sysIdCommand.battery);
            telemetry.addData("Position (rad)", sysIdCommand.position);
            telemetry.addData("Window speed (rad/s)", sysIdCommand.windowSpeed);
            telemetry.addLine("X to abort");
            telemetry.update();
            return;
        }

        if (!promptsDone) {
            telemetry.addLine("Prompts were not completed - stop and re-init");
        } else if (!testStarted) {
            telemetry.addData("Ready", selectedRun);
            telemetry.addLine("Make sure the flywheel is stopped and no balls are loaded");
            telemetry.addLine("Press A to start (X aborts)");
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
            motor.setRunMode(Motor.RunMode.RawPower); // open loop, no velocity controller
            motor.setInverted(MOTOR_REVERSED);
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            motor.resetEncoder();
        }

        void setPower(double power) {
            motor.set(power);
        }

        void stop() {
            motor.stopMotor();
        }

        /** Slow call: throttle it. */
        double getBatteryVoltage() {
            return voltageSensor.getVoltage();
        }

        /** Continuous position of the selected shaft (see SHAFT_REVS_PER_MOTOR_REV), in radians. */
        double getPositionRad() {
            return encSign * motor.getCurrentPosition() * (2 * Math.PI / motor.getCPR()) * SHAFT_REVS_PER_MOTOR_REV;
        }

        double getVelocityRpm() {
            double motorTPS = motor.getCorrectedVelocity();
            return (motorTPS * 60) / motor.getCPR();
        }
    }

    // ======================= COMMAND =======================

    /** Runs one full test. Finishes when done; interrupted (X / OpMode stop) means nothing is saved. */
    private class SysIdCommand extends CommandBase {
        private final List<double[]> records = new ArrayList<>(6000);

        // exposed for telemetry
        Phase phase = Phase.REST;
        int stage;
        double t, battery, position, power, windowSpeed;

        private double[] powers; // fixed power per step, computed once from the starting battery voltage
        private double startPower; // fixed power for the unrecorded pre-spin (0 = no start voltage)
        private boolean rejected, timedOut;

        private long startNanos;
        private double lastBatteryReadT, batteryT;
        private double lastPower, commandChangeT;
        private double stageStartT, settledT;

        // settling detection (average speed over successive windows of the position)
        private boolean windowOpen;
        private double winT0, winPos0, prevWindowSpeed;
        private int stableCount;

        SysIdCommand() {
            addRequirements(flywheel);
        }

        @Override
        public void initialize() {
            records.clear();
            rejected = false;
            timedOut = false;
            stage = 0;
            settledT = 0;
            lastPower = Double.NaN; // the first command sent always counts as a command change
            commandChangeT = 0;
            power = 0;
            position = 0;

            startNanos = System.nanoTime();
            double before = elapsed();
            battery = flywheel.getBatteryVoltage();
            batteryT = 0.5 * (before + elapsed());
            lastBatteryReadT = batteryT;

            // Fixed power for each step: p = V_target / V_battery_start, kept constant for the whole step.
            powers = new double[selectedRun.volts.length];
            for (int i = 0; i < powers.length; i++) {
                powers[i] = selectedRun.volts[i] / battery;
                if (powers[i] > 1.0) {
                    rejected = true;
                    status = String.format(
                            Locale.US,
                            "Battery %.2f V is too low for the %.1f V target (power would be %.2f). Choose a lower target.",
                            battery,
                            selectedRun.volts[i],
                            powers[i]);
                }
            }

            // Optional start voltage (pre-spin): same fixed-power rule. Also used for the rest baseline, if any.
            startPower = selectedRun.startVolts / battery;
            if (startPower > 1.0) {
                rejected = true;
                status = String.format(
                        Locale.US,
                        "Battery %.2f V is too low for the %.1f V start voltage (power would be %.2f). Choose a lower start.",
                        battery,
                        selectedRun.startVolts,
                        startPower);
            }

            if (selectedRun.startVolts > 0) {
                phase = Phase.PRESPIN;
                beginStep(0);
            } else if (selectedRun.restFirst) {
                phase = Phase.REST;
            } else {
                phase = Phase.SPINUP;
                beginStep(0);
            }
        }

        @Override
        public void execute() {
            if (rejected) return;

            double now = elapsed();

            // Supply voltage is read separately (slow call), so it carries its own timestamp.
            if (now - lastBatteryReadT >= VOLTAGE_REFRESH_S) {
                battery = flywheel.getBatteryVoltage();
                batteryT = 0.5 * (now + elapsed());
                lastBatteryReadT = batteryT;
            }

            // ---- phase transitions ----
            if (phase == Phase.REST && now >= PRE_REST_S) {
                phase = Phase.SPINUP;
                beginStep(now);
            } else if (phase == Phase.HOLD && now - settledT >= HOLD_S) {
                if (stage + 1 < powers.length) {
                    stage++;
                    phase = Phase.SPINUP;
                    beginStep(now);
                } else {
                    phase = Phase.DONE;
                }
            }
            if (phase == Phase.DONE) return;

            // ---- command: fixed power per step, timestamp taken right before it is sent ----
            // PRESPIN and REST use the start power (0 when the run has no start voltage).
            power = (phase == Phase.SPINUP || phase == Phase.HOLD) ? powers[stage] : startPower;
            if (power != lastPower) commandChangeT = elapsed();
            flywheel.setPower(power);
            lastPower = power;

            // ---- sample (position is the first hardware read after the bulk cache was cleared) ----
            position = flywheel.getPositionRad();
            t = elapsed();
            if (phase != Phase.PRESPIN) { // the pre-spin is never recorded
                records.add(new double[] {t, position, power, battery, batteryT, commandChangeT});
            }

            if (phase == Phase.SPINUP || phase == Phase.PRESPIN) detectSettling();
        }

        /** Average speed over ~0.5 s windows: compare successive windows, require a flat trend for a while. */
        private void detectSettling() {
            if (!windowOpen) {
                windowOpen = true;
                winT0 = t;
                winPos0 = position;
            } else if (t - winT0 >= SETTLE_WINDOW_S) {
                double w = (position - winPos0) / (t - winT0);
                windowSpeed = w;

                if (!Double.isNaN(prevWindowSpeed)) {
                    boolean stable = Math.abs(w) > MIN_SETTLED_VEL
                            && Math.abs(prevWindowSpeed) > MIN_SETTLED_VEL
                            && Math.abs(w - prevWindowSpeed) <= SETTLE_TOL * Math.abs(prevWindowSpeed);
                    stableCount = stable ? stableCount + 1 : 0;
                }

                prevWindowSpeed = w;
                winT0 = t;
                winPos0 = position;

                if (stableCount >= SETTLE_STABLE_COMPARISONS) {
                    if (phase == Phase.PRESPIN) {
                        finishPreSpin();
                    } else {
                        phase = Phase.HOLD;
                        settledT = t;
                    }
                    return;
                }
            }

            if (t - stageStartT > MAX_SPINUP_S) {
                if (phase == Phase.PRESPIN) {
                    rejected = true; // nothing has been recorded yet, so there is nothing to save
                    status = "Start voltage did not settle within MAX_SPINUP_S - nothing saved";
                } else {
                    timedOut = true;
                }
                phase = Phase.DONE;
            }
        }

        /** The start voltage has settled: re-zero the clock so recording begins now, then continue as usual. */
        private void finishPreSpin() {
            double shift = elapsed();
            startNanos += (long) (shift * 1e9);
            // Timestamps taken before recording started now lie slightly in the past (negative), which is accurate.
            lastBatteryReadT -= shift;
            batteryT -= shift;
            commandChangeT -= shift;

            if (selectedRun.restFirst) {
                phase = Phase.REST; // baseline recorded at the start voltage
            } else {
                phase = Phase.SPINUP;
                beginStep(elapsed());
            }
        }

        private void beginStep(double now) {
            stageStartT = now;
            windowOpen = false;
            prevWindowSpeed = Double.NaN;
            windowSpeed = Double.NaN;
            stableCount = 0;
        }

        /** Monotonic seconds since the command was initialized. */
        private double elapsed() {
            return (System.nanoTime() - startNanos) * 1e-9; // toSeconds
        }

        @Override
        public boolean isFinished() {
            return rejected || phase == Phase.DONE;
        }

        @Override
        public void end(boolean interrupted) {
            flywheel.stop(); // stop first; only then touch the disk
            /*
            if (selectedRun.fullStop) {
                while (flywheel.getVelocityRpm() > VEL_MOTOR_STOPPED_RPM) {
                    telemetry.addLine("Waiting for motor stop");
                    telemetry.addData("Current RPM", flywheel.getVelocityRpm());
                    telemetry.update();
                }
                telemetry.clearAll();
                telemetry.update();
            }
             */



            if (rejected) return; // status already explains why
            if (interrupted) {
                status = "Aborted - nothing saved";
            } else {
                status = timedOut
                        ? "Did not settle within MAX_SPINUP_S - raise it and repeat (saving anyway)"
                        : "Test complete";

                File file = saveData(records);
                if (!file.exists()) {
                    return;
                }

                Pair<Integer, String> res = null;

                if (prompter.get("upload")) {
                    res = uploadFile(file);
                    if (res != null) {
                        updateStatusFromResponse(res);
                    }
                }

                if (prompter.getOrDefault("remove", false)) {
                    if (res != null) {
                        if (res.first < 0 || (res.first >= 200 && res.first <= 299)) {
                            file.delete();
                        } else {
                            status = status + "\nSkipping file deletion due to failed upload";
                        }
                    }
                }
            }
        }
    }

    // ======================= SAVING =======================

    @SuppressLint("SdCardPath")
    private File saveData(List<double[]> records) {
        File dir = new File(SAVE_DIR);
        dir.mkdirs();
        File file = new File(dir, "sysid_" + selectedRun.id + "_" + System.currentTimeMillis() + ".csv");

        CSVFormat csvFormat = CSVFormat.DEFAULT
                .builder()
                .setHeader(
                        "time_s",
                        "position_rad",
                        "power_command",
                        "battery_voltage_V",
                        "battery_time_s",
                        "command_change_time_s")
                .setAutoFlush(false)
                .get();

        try (CSVPrinter printer = new CSVPrinter(new FileWriter(file), csvFormat)) {
            for (double[] r : records) {
                printer.printRecord(r[0], r[1], r[2], r[3], r[4], r[5]);
            }
            printer.flush();
            status += " | Saved " + file.getName() + " (" + records.size() + " samples)";
        } catch (IOException e) {
            status = "SAVE FAILED: " + e.getMessage();
        }

        return file;
    }

    private Pair<Integer, String> uploadFile(File file) {
        OkHttpClient client = new OkHttpClient();
        RequestBody requestBody = RequestBody.create(file, MediaType.get("text/csv"));
        Request request = new Request.Builder()
                .url(SERVER.newBuilder().addPathSegment(file.getName()).build())
                .put(requestBody)
                .build();

        try(Response response = client.newCall(request).execute()){
            return new Pair<>(response.code(), response.body().string());
        } catch (IOException e) {
            Log.e("SysID", "Upload to server failed", e);
        }

        return null;
    }

    private void updateStatusFromResponse(Pair<Integer, String> pair) {
        if (pair.first >= 200 && pair.first <= 299) {
            status = status + "\nUpload successful";
        } else {
            status = status + "Uploaded exited with error code" + pair.first;
            Log.e("SysID", "Upload failed: " + pair.second);
        }
    }
}
