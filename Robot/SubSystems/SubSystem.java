package org.firstinspires.ftc.teamcode.FTC_Library.Robot.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Subsystem interface that all subsystems should implement
 */

public abstract class SubSystem {

    /**
     * @param hardwareDevices map provided by FTC SDK
     * @return if it was initialized successfully
     */
    abstract public boolean init(HardwareMap hardwareDevices);

    /**
     * Run when stopping the subsystem
     */
    public void stop() {
    }

    /**
     * Run every tick
     */
    public void tick() {
    }

    /**
     * Can be overridden if needed to change name
     *
     * @return ID of class
     */
    public String ID() {
        return "";
    }

    /**
     * Can be overridden by user to provide custom functionality
     *
     * @return if system is initialized
     */
    public boolean isInitialized() {
        return true;
    }

    /**
     * Can be overridden by user to provide custom functionality
     *
     * @return if system is functioning
     */
    public boolean isFunctioning() {
        return true;
    }
}
