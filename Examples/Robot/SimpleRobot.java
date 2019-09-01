package org.firstinspires.ftc.teamcode.FTC_Library.Examples.Robot;

import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.FTC_Library.Examples.Robot.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.FTC_Library.Robot.RobotBase;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Just a simple robot that drives
 */

public class SimpleRobot extends RobotBase {
    public Drive drive = new Drive()
            .setMotorNames("Left_Motor", "Right_Motor");

    public SimpleRobot() {
        addSubSystem(drive);
    }
}
