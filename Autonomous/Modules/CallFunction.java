package org.firstinspires.ftc.teamcode.FTC_API.Autonomous.Modules;

import org.firstinspires.ftc.teamcode.FTC_API.Options;

/**
 * Created by ethan.hampton on 11/2/2017.
 * This allows you to call a function in an autonomous program. Designed to work simplistically and easily
 */

public class CallFunction extends Module {


    public boolean setFunction() {

        return true;
    }

    @Override
    public void start() {

    }

    @Override
    public void tick() {

    }


    @Override
    public Options options() {
        return null;
    }

    @Override
    public String ID() {
        return super.ID();
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public String[] requiredSubSystems() {
        return new String[0];
    }
}
