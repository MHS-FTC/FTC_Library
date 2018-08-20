package org.firstinspires.ftc.teamcode.FTC_API.Robot.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Subsystem interface that all subsystems should implement
 */

public abstract class SubSystem {

    abstract public boolean init(HardwareMap hardwareDevices);

    public void stop() {
    }

    public void tick() {
    }

    public String ID() {
        return "";
    }

    public boolean isInitialized() {
        return true;
    }

    public boolean isFunctioning() {
        return true;
    }
}
