package org.firstinspires.ftc.teamcode.FTC_API.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Ethan Hampton on 8/19/17.
 *
 * Shows example of how to do a simple telop method
 */
@Disabled
@TeleOp(name = "SimpleTeleop")
public class SimpleTeleop extends OpMode{
    private SimpleRobot robot = new SimpleRobot();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.start();
    }

    @Override
    public void loop() {
        robot.tick();

        robot.drive.drive(gamepad1.left_stick_y,gamepad1.right_stick_y);
    }
}
