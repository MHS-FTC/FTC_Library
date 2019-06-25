package org.firstinspires.ftc.teamcode.FTC_Library.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Module;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Wait;
import org.firstinspires.ftc.teamcode.FTC_Library.Robot.RobotBase;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Base autonomous class that manages modules and running them on time
 */

abstract public class AutonomousBase extends OpMode {
    public RobotBase robot;
    private Module[][] steps;
    private int currentStep = 0;//zero indexed
    private int currentOption = 0;
    private int totalSteps;
    private boolean isDone = false;

    private boolean isFirstLoop = true;

    /**
     * Initializes the Autonomous system
     *
     * @param map   Hardware Map from FTC SDK
     * @param robot The robot to use for this autonomous
     * @param steps The list of steps to be run during this autonomous
     */
    protected void init(HardwareMap map, RobotBase robot, Module[][] steps) {
        robot.init(map);//starts and initializes the robot
        this.steps = steps;
        totalSteps = steps.length;
        //This was too hard
        this.robot = robot;
    }

    abstract public void init();

    /**
     * This is called by FTC SDK and runs each module as well as some user code
     */
    @Override
    public void loop() {
        tick();//run user code
        robot.tick();//ticks the robot automatically

        if (!isDone) {
            //old way of doing things
            Module current = steps[currentStep][currentOption];// loads current running module
            if (isFirstLoop) {
                current.init(robot, 0, telemetry);
                current.start();
                isFirstLoop = false;
            }

            boolean currentIsDone = current.tick();//run the current module (the return type tells us if it is done)

            if (currentIsDone) {//if the current module is done
                currentStep++;//get new module, start and initialize it

                currentOption = current.stop();//stop current module and get what option the module wants for next step
                int selectedOption = currentOption;//insure the selected option is passed through to next module even if that isn't the one selected

                if (currentStep <= totalSteps - 1) {//insures we have not gone through all our steps
                    int maxOption = steps[currentStep].length - 1;//get amount of modules currently available in the next step
                    if (currentOption > maxOption || currentOption < 0) {//check if next option is valid
                        currentOption = 0;//otherwise use default of 0
                    }

                    current = steps[currentStep][currentOption];//get new current module

                    if (current == null) {//redefine null modules as a 1 milli wait
                        current = steps[currentStep][currentOption] = new Wait().setWaitTime(1);
                    }
                    current.init(robot, selectedOption, telemetry);//initialize it with the passed through option so it can be passed through multiple times
                    current.start();//start current module
                } else {
                    isDone = true;
                }
            }

        }
    }

    /**
     * This is the method that should be overridden by the user to do stuff every loop.
     * Is called by this class every "tick"
     */
    public void tick() {

    }

    public RobotBase getRobot() {
        return robot;
    }
}
