package org.firstinspires.ftc.teamcode.FTC_API.Examples;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.FTC_API.Robot.Robot;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Just a simple robot that drives
 */

public class SimpleRobot extends Robot {
    Drive drive = new Drive()
            .setMotorNames("Left_Motor", "Right_Motor")
            .setMotorType(MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class));

    public SimpleRobot() {
        addSubSystem(drive);
    }

}
