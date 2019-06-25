package org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules;

import org.firstinspires.ftc.teamcode.FTC_Library.Robot.SubSystems.SidedDriveSystemTemplate;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Simple drive for time module
 */

public class DriveTime extends Module {
    private SidedDriveSystemTemplate drive;
    private double startTime;

    private double leftSpeed;
    private double rightSpeed;
    private int driveTime;

    @Override
    public void start() {
        drive = robot.getSidedDriveSystem();
        startTime = robot.getTimeMilliseconds();

        drive.driveTank(leftSpeed, rightSpeed);
    }

    @Override
    public boolean tick() {
        if ((robot.getTimeMilliseconds() - startTime) > driveTime) {
            drive.driveTank(0, 0);
            return true;
        }
        return false;
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
