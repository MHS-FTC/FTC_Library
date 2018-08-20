package org.firstinspires.ftc.teamcode.FTC_API.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.FTC_API.Autonomous.Modules.Module;
import org.firstinspires.ftc.teamcode.FTC_API.Robot.RobotBase;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Base autonomous class that manages modules and running them on time
 */

abstract public class AutonomousBase extends OpMode{
    public RobotBase robot;
    private Module[][] steps;
    private int currentStep = 0;//zero indexed
    private int currentPosition = 0;
    private int totalSteps;
    private boolean isDone = false;

    private boolean isFirstLoop = true;

    protected void init(HardwareMap map, RobotBase robot, Module[][] steps) {
        robot.init(map);//starts and initializes the robot
        robot.start();
        this.steps = steps;
        totalSteps = steps.length;
        //This was too hard
        this.robot = robot;
    }

    abstract public void init();

    public void loop() {
        tick();//run user code
        robot.tick();//ticks the robot automatically

        if (!isDone) {
            //old way of doing things
            Module current = steps[currentStep][currentPosition];// loads current running module
            if (isFirstLoop) {
                current.init(robot, 0, telemetry);
                current.start();
                isFirstLoop = false;
            }
            current.tick();//runs tick for current module
            if (current.isDone()) {//if the current module is done
                currentStep++;//get new module, start and initialize it

                currentPosition = 0;
                currentPosition = current.stop();//stop it and get the where the module wants the next step to go
                int maxPosition = steps[currentStep].length - 1;//get amount of modules currently available in the next step
                int position;
                if (currentPosition > maxPosition) {//if the current position does not exist then set it to 0
                    position = 0;
                } else {
                    position = currentPosition;//otherwise just use the position given
                }
                if (currentStep <= (totalSteps - 1)) {//insures we have not gone through all our steps
                    current = steps[currentStep][position];
                    current.init(robot, currentPosition, telemetry);//initialize it with the passed through position so it can be passed through multiple times
                    current.start();
                } else {
                    isDone = true;
                }
                currentPosition = position;//reset position, should be stored by module. This way we don't get any null errors when there is only one module to choose from
            }
        }
    }

    //can be overridden by the user to do stuff every loop
    public void tick(){

    }

    public RobotBase getRobot() {
        return robot;
    }
}
