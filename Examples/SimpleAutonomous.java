package org.firstinspires.ftc.teamcode.FTC_Library.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.AutonomousBase;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.CallFunction;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.DriveTime;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Module;

/**
 * Created by Ethan Hampton on 8/19/17.
 * Example Autonomous
 */
@Disabled
@Autonomous(name = "Simple Autonomous")
public class SimpleAutonomous extends AutonomousBase {
    private SimpleRobot bot = new SimpleRobot();
    private final Module[][] steps = new Module[][]{
            {new DriveTime().setSpeeds(1, 1).setTime(2000)},
            {new DriveTime().setSpeeds(0.5, 0.9).setTime(2500)},
            {new CallFunction().setFunction(() -> bot.drive.drive(1, 1))},
            {new CallFunction().setFunction(() -> bot.drive.drive(0, 0))},


            {new RandomMultiPosibility()},
            {new DriveTime().setSpeeds(1, 1).setTime(2500), new DriveTime().setSpeeds(0.5, 1).setTime(500)},
    };

    @Override
    public void init() {
        init(hardwareMap, bot, steps);
    }

    @Override
    public void tick(){
        telemetry.addLine("Wow this works!");
    }
}
