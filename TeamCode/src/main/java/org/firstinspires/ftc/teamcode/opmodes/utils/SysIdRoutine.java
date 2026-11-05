package org.firstinspires.ftc.teamcode.opmodes.utils;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;

import java.io.FileWriter;
import java.io.IOException;

public class SysIdRoutine extends CommandOpMode {
    private FileWriter fileWriter;
    private CSVPrinter csvPrinter;
    private GamepadEx gamepad;
    private MotorEx flywheel;
    private VoltageSensor voltageSensor;

    @SuppressLint("SdCardPath")
    @Override
    public void initialize() {
        CSVFormat csvFormat = CSVFormat.DEFAULT.builder()
                .setHeader("time_s", "voltage_V", "position_rad", "voltage_read_time_s")
                .get();

        try {
            fileWriter = new FileWriter("/sdcard/FIRST/sys_id_data.csv");
            csvPrinter = new CSVPrinter(fileWriter, csvFormat);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        gamepad = new GamepadEx(gamepad1);
        flywheel = new MotorEx(hardwareMap, "flywheel", Motor.GoBILDA.BARE);
        flywheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void run() {
        double voltage = voltageSensor.getVoltage();

        flywheel.set(Double.NaN); //TODO: set voltage to flywheel
        double time = getRuntime();
        double position = ticksToRad(flywheel.getCurrentPosition());

        try {
            csvPrinter.printRecord(time, voltage, position);
            csvPrinter.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void end() {
        try {
            csvPrinter.close();
            fileWriter.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private void setMotorVoltage(double voltage) {

    }

    private double ticksToRad(double ticks) {
        return ticks * (2 * Math.PI / flywheel.getCPR());
    }
}
