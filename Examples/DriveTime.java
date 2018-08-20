package org.firstinspires.ftc.teamcode.FTC_API.Examples;

import org.firstinspires.ftc.teamcode.FTC_API.Autonomous.Modules.Module;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Simple drive for time module
 */

class DriveTime extends Module {
    private Drive drive;
    private boolean isDone = false;
    private double startTime;

    private double leftSpeed;
    private double rightSpeed;
    private int driveTime;

    @Override
    public void start() {
        drive = (Drive) robot.getSubSystem(Drive.ID);
        startTime = robot.getTimeMilliseconds();

        drive.drive(leftSpeed, rightSpeed);
    }

    @Override
    public void tick() {
        if ((robot.getTimeMilliseconds() - startTime) > driveTime) {
            drive.drive(0, 0);
            isDone = true;
        }
    }

    @Override
    public boolean isDone() {
        return isDone;
    }

    @Override
    public String[] requiredSubSystems() {
        return new String[]{Drive.ID};
    }


    public DriveTime setSpeeds(double left, double right) {
        leftSpeed = left;
        rightSpeed = right;
        return this;
    }

    public DriveTime setTime(int time) {
        driveTime = time;
        return this;
    }
}
