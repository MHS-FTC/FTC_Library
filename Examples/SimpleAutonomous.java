package org.firstinspires.ftc.teamcode.API.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.API.Autonomous.AutonomousBase;
import org.firstinspires.ftc.teamcode.API.Autonomous.Modules.Module;

/**
 * Created by Ethan Hampton on 8/19/17.
 * Example Autonomous
 */
@Disabled
@Autonomous(name = "Simple Autonomous")
public class SimpleAutonomous extends OpMode {
    private AutonomousBase auto = new AutonomousBase();
    private SimpleRobot bot = new SimpleRobot();
    private final Module[][] steps = new Module[][]{
            {new DriveTime().setSpeeds(1, 1).setTime(2000)},
            {new DriveTime().setSpeeds(0.5, 1).setTime(2500)},


            {new RandomMultiPosibility()},
            {new DriveTime().setSpeeds(1, 1).setTime(2500), new DriveTime().setSpeeds(0.5, 1).setTime(500)},
    };

    @Override
    public void init() {
        auto.init(hardwareMap, bot, steps);
    }

    @Override
    public void loop() {
        auto.loop();

        telemetry.addData("Test", bot.drive.leftMotor.getPower());
    }
}
