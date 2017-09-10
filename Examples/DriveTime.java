package org.firstinspires.ftc.teamcode.API.Examples;

import org.firstinspires.ftc.teamcode.API.Autonomous.Modules.Module;
import org.firstinspires.ftc.teamcode.API.Options;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Simple drive for time module
 */

public class DriveTime extends Module {
    private Options options = new Options("Drive For Time");
    private Drive drive;
    private boolean isDone = false;
    private double startTime;

    @Override
    public void start() {
        drive = (Drive) robot.getSubSystem(Drive.ID);
        startTime = robot.getTimeMilliseconds();

        drive.drive(Double.valueOf(options.get("left_speed")), Double.valueOf(options.get("right_speed")));
    }

    @Override
    public void tick() {
        if ((robot.getTimeMilliseconds() - startTime) > Integer.getInteger(options.get("drive_time"))) {
            drive.drive(0, 0);
            isDone = true;
        }
    }

    @Override
    public Options options() {
        return options;
    }

    @Override
    public boolean isDone() {
        return isDone;
    }

    @Override
    public String[] requiredSubSystems() {
        return new String[]{Drive.ID};
    }


    public DriveTime setSpeeds(double leftSpeed, double rightSpeed) {
        options.add("left_speed", String.valueOf(leftSpeed));
        options.add("right_speed", String.valueOf(rightSpeed));
        return this;
    }

    public DriveTime setTime(int time) {
        options.add("drive_time", String.valueOf(time));
        return this;
    }
}
