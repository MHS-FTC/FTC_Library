package org.firstinspires.ftc.teamcode.FTC_Library.Examples;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.FTC_Library.Robot.RobotBase;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Just a simple robot that drives
 */

class SimpleRobot extends RobotBase {
    Drive drive = new Drive()
            .setMotorNames("Left_Motor", "Right_Motor")
            .setMotorType(MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class));

    SimpleRobot() {
        addSubSystem(drive);
    }
}
