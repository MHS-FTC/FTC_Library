package org.firstinspires.ftc.teamcode.API.Robot.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.API.Options;

/**
 * Created by Ethan Hampton on 8/19/17.
 *
 * Subsystem interface that all subsystems should implement
 */

public abstract class SubSystem {

    abstract public boolean init(HardwareMap hardwareDevices);

    public void start(){}

    public void stop(){}

    public void tick(){}

    abstract public Options options();

    /**
     * Gives user a chance to set up options before initialization
     */
    public void setOptions(){}

    public String ID(){
        return options().getName();
    }

    public boolean isInitialized(){
        return true;
    }

    public boolean isFunctioning(){
        return true;
    }
}
