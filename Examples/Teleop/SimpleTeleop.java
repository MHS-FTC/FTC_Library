package org.firstinspires.ftc.teamcode.FTC_Library.Examples.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC_Library.Examples.Robot.SimpleRobot;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Shows example of how to do a simple teleop method
 */
@Disabled
@TeleOp(name = "SimpleTeleop")
public class SimpleTeleop extends OpMode {
    private SimpleRobot robot = new SimpleRobot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.tick();

        robot.drive.drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
    }
}
