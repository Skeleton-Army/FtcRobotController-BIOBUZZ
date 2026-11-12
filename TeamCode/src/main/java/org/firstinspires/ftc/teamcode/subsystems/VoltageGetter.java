package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.skeletonarmy.marrow.OpModeManager;

public class VoltageGetter extends SubsystemBase {
    private static VoltageGetter INSTANCE = null;
    private final OpMode opMode;
    private final VoltageSensor voltageSensor;
    private double cachedVoltage;

    private VoltageGetter(final HardwareMap hardwareMap, final OpMode opMode) {
        super();
        this.opMode = opMode;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        cachedVoltage = voltageSensor.getVoltage();
    }

    @Override
    public void periodic() {
        cachedVoltage = voltageSensor.getVoltage();
    }

    public static VoltageGetter getInstance() {
        OpMode currentOpMode = OpModeManager.getActiveOpMode();
        if (INSTANCE == null || INSTANCE.opMode != currentOpMode) {
            INSTANCE = new VoltageGetter(OpModeManager.getHardwareMap(), currentOpMode);
        }

        return INSTANCE;
    }

    public double getVoltage() {
        return cachedVoltage;
    }
}
