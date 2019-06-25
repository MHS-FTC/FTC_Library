package org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules;

/**
 * Created by Ethan Hampton on 24-Jun-2019.
 * Designed to wait for a certain amount of time
 */

public class Wait extends Module {
    private double startTime;
    private long waitTime = 1000;//default wait time of one second

    public Wait setWaitTime(long waitTime) {
        this.waitTime = waitTime;
        return this;
    }

    @Override
    public void start() {
        startTime = robot.getTimeMilliseconds();
    }

    @Override
    public boolean tick() {
        return (startTime + waitTime) < robot.getTimeMilliseconds();
    }
}