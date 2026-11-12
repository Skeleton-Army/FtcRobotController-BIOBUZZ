package org.firstinspires.ftc.teamcode.opmodes.utils;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.skeletonarmy.marrow.BulkCaching;
import com.skeletonarmy.marrow.LynxUtil;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;

import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

@Utility
public class SysIdRoutine extends CommandOpMode {
    private CSVPrinter csvPrinter;
    private GamepadEx gamepad;
    private MotorEx flywheel;
    private VoltageSensor voltageSensor;
    private final List<List<Object>> records = new ArrayList<>(100);

    @SuppressLint("SdCardPath")
    @Override
    public void initialize() {
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        CSVFormat csvFormat = CSVFormat.DEFAULT.builder()
                .setHeader("time_s", "voltage_V", "position_rad", "voltage_read_time_ms")
                .setAutoFlush(false)
                .get();

        try {
            Writer fileWriter = new FileWriter("/sdcard/sys_id_data.csv");
            csvPrinter = new CSVPrinter(fileWriter, csvFormat);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        gamepad = new GamepadEx(gamepad1);
        flywheel = new MotorEx(hardwareMap, "flywheel", Motor.GoBILDA.BARE);
        //flywheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        gamepad.getGamepadButton(GamepadKeys.Button.A).whileActiveContinuous(runTest()).whenInactive(() -> {
            flywheel.set(0);
        });
        gamepad.getGamepadButton(GamepadKeys.Button.B).whenActive(this::saveData);
    }

    @Override
    public void run() {
        super.run();
        LynxUtil.clearBulkCache(hardwareMap);
    }

    private double ticksToRad(double ticks) {
        return ticks * (2 * Math.PI / flywheel.getCPR());
    }

    private void addRecord(Object... values) {
        records.add(Arrays.asList(values));
    }

    private void saveData() {
        try {
            for (List<Object> row : records) {
                csvPrinter.printRecord(row);
            }
            csvPrinter.flush();
            csvPrinter.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }



    private Command runTest() {
        return new InstantCommand(() -> {
            telemetry.addLine("Press A to stop test");

            double preRead  = System.currentTimeMillis();
            double voltage = voltageSensor.getVoltage();
            double postRead = System.currentTimeMillis();

            double desired = 10;
            flywheel.set(-(desired/voltage));
            double position = ticksToRad(flywheel.getCurrentPosition());
            telemetry.addData("Voltage", voltage);
            telemetry.addData("Sent Voltage", -(desired/voltage));
            telemetry.addData("Position", position);

            telemetry.update();

            addRecord(getRuntime(), voltage, position, postRead - preRead);
        });
    }
}
