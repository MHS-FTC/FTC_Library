package org.firstinspires.ftc.teamcode.FTC_API.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTC_API.Autonomous.Modules.Module;
import org.firstinspires.ftc.teamcode.FTC_API.Robot.RobotBase;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Base autonomous class that manages modules and running them on time
 */

public class AutonomousBase {
    public RobotBase robot;
    private Telemetry telemetry = null;
    private Module[][] steps;
    private int currentStep = 0;//zero indexed
    private int currentPosition = 0;
    private int totalSteps;
    private boolean isDone = false;

    private boolean isFirstLoop = true;

    public void init(HardwareMap map, RobotBase robot, Module[][] steps) {
        robot.init(map);//starts and initializes the robot
        robot.start();
        this.steps = steps;
        totalSteps = steps.length;
        //This was too hard
        this.robot = robot;
    }

    public void loop() {
        robot.tick();//ticks the robot automatically

        /*
        //way with async, don't really want to mess with this right now
        for (Module m : currentlyRunning) {//loop through currently running modules
            m.tick();
            if (m.isDone()) {//if the current module is done
                m.stop();//stop it
                currentlyRunning.remove(m);//remove the module from the list of currently running modules
            }
        }

        if (currentlyRunning.isEmpty()) {
            currentStep++;
            Collections.addAll(currentlyRunning, steps[currentStep]);//add all things to be done in async(or not) to the currently running list

            //if we are trying to run more than one module at a time then make sure they are all compatible
            // TODO: 9/3/17 Add checks to insure they are not using any of the same subsystems
            if (currentlyRunning.size() > 1) {
                for (Module m : currentlyRunning) {
                    if (!m.options().get(Option.IS_ASYNC).equals("true")) {
                        throw new UnsupportedOperationException("Module: " + m.options().getName() + " is not an ASYNC module and can not be run at the same time as other modules");
                    }
                }
            }

            //init and start all modules
            for (Module m : currentlyRunning) {
                m.init(robot);
                m.start();
            }
        }
        */

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

    public RobotBase getRobot() {
        return robot;
    }

    public AutonomousBase setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
        return this;
    }

}
