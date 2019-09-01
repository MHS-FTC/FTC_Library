package org.firstinspires.ftc.teamcode.FTC_Library.Examples.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.AutonomousBase;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Module;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Premade.Gyroscope;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Premade.PIDEncoderDrive;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Premade.PIDEncoderDrive.PIDConfig;
import org.firstinspires.ftc.teamcode.FTC_Library.Examples.Robot.SimpleRobot;


@Disabled
@Autonomous(name = "Autonomous With PID Control Example")
public class AutonomousWithPIDControl extends AutonomousBase {
    private SimpleRobot bot = new SimpleRobot();

    //should be different for each robot
    private PIDEncoderDrive.PIDConfig config = new PIDConfig()
            .setPID(0.00205, 0.0004, 0.00061)//see http://www.georgegillard.com/documents/2-introduction-to-pid-controllers
            .setSettlingTime(2.4)//in seconds, how long does it have to be at the correct position for before it moves on, don't worry too much
            .setTolerance(200)//how many encoder ticks does it need to be within to be accurate
            .setMaxSpeed(0.75)//max speed of the robot
            .setWheelCircumference(12.56);//how big the wheels are (for distance calculations

    private Module[][] steps = new Module[][]{
            {new PIDEncoderDrive().setDistances(12, 12).setConfig(config)},//go forward 12 inches
            {new Gyroscope().setTurn(90)},//turn 90 degrees to the right (right is positive, left is negative)
            {new PIDEncoderDrive().setDistances(12, 12).setConfig(config)},//go forward 12 inches
    };


    @Override
    public void init() {
        //make sure gyroscope module has a gyroscope to deal with. This isn't totally ideal, but it will have to do.
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro");
        Gyroscope.setGyro(gyro);


        init(hardwareMap, bot, steps);
    }
}
