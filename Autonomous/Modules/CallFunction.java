package org.firstinspires.ftc.teamcode.FTC_API.Autonomous.Modules;

import org.firstinspires.ftc.teamcode.FTC_API.Options;

/**
 * Created by ethan.hampton on 11/2/2017.
 * This allows you to call a function in an autonomous program. Designed to work simplistically and easily.
 * This passes though it's number in the array to keep things simple, you can reset it by calling <code>resetArrayPosition()</code> when building
 */

public class CallFunction extends Module {
    public interface Command {
        //Ideally we override this method using lambda and then are able to execute it via autonomous stuff. This means
        //we can run basically anything we can run in teleop in autonomous
        void apply();
    }

    private Command command;
    private Options options = new Options("Call Functions");

    public CallFunction setFunction(Command cmd) {
        command = cmd;
        return this;
    }

    @Override
    public void start() {
        command.apply();
    }

    @Override
    public void tick() {

    }

    @Override
    public Options options() {
        return options;
    }

    @Override
    public String ID() {
        return "Call Functions";
    }

    @Override
    public boolean isDone() {
        return true;
    }

    @Override
    public String[] requiredSubSystems() {
        return new String[0];
    }
}
