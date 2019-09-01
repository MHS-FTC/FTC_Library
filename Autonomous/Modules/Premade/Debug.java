package org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Premade;

import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Module;

/**
 * Allows users to set a message to be shown on phone log for debugging of autonomous
 */
public class Debug extends Module {
    private String message;
    @Override
    public void start() {
        telemetry.log().add(message);
    }

    @Override
    public boolean tick() {
        return true;
    }

    public Debug setMessage(String message) {
        this.message = message;
        return this;
    }
}
