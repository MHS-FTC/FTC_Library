package org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules;

/**
 * Created by ethan.hampton on 11/2/2017.
 * This allows you to call a function in an autonomous program. Designed to work simplistically and easily.
 * This passes though it's number in the array to keep things simple, you can reset it by calling <code>resetArrayPosition()</code> when building
 */

public class CallFunction extends Module {
    public interface Command {
        void apply();
    }

    private Command command;

    /**
     *
     * @param cmd use lambda to execute any command needed
     * @return this, used to chain
     */
    public CallFunction setFunction(Command cmd) {
        command = cmd;
        return this;
    }

    @Override
    public void start() {
        command.apply();
    }

    @Override
    public boolean tick() {
        return true;
    }
}
