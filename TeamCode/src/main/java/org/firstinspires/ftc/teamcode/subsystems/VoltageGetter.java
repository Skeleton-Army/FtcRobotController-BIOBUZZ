package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.skeletonarmy.marrow.OpModeManager;

public class VoltageGetter extends SubsystemBase {
    private static VoltageGetter INSTANCE = null;
    private final VoltageSensor voltageSensor;
    private double cachedVoltage;
    private VoltageGetter(final HardwareMap hardwareMap) {
        super();
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        cachedVoltage = voltageSensor.getVoltage();
    }

    @Override
    public void periodic() {
        cachedVoltage = voltageSensor.getVoltage();
    }

    public static VoltageGetter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VoltageGetter(OpModeManager.getHardwareMap());
        }

        return INSTANCE;
    }

    public double getVoltage() {
        return cachedVoltage;
    }
}
