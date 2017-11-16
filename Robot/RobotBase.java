package org.firstinspires.ftc.teamcode.FTC_API.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTC_API.Option;
import org.firstinspires.ftc.teamcode.FTC_API.Robot.SubSystems.SubSystem;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Main robot class that should be extended by all robot configurations
 */

public class RobotBase {

    private HashMap<String, SubSystem> subSystems = new HashMap<String, SubSystem>();
    private HashSet<String> needsTick = new HashSet<String>();

    private ElapsedTime time = new ElapsedTime();

    protected void addSubSystem(SubSystem sub) {
        subSystems.put(sub.options().getName(), sub);
    }

    public Collection<SubSystem> getSubSystems(){
        return subSystems.values();
    }

    public SubSystem getSubSystem(String name){
        return subSystems.get(name);
    }

    //registers all of the subsystems and prepares the robot
    public boolean init(HardwareMap hardwareMap) {
        boolean noErrors = true;
        for (SubSystem s : subSystems.values()) {
            s.setOptions();
            s.init(hardwareMap);
        }
        for (SubSystem s : subSystems.values()) {
            if (s.isFunctioning() && s.isInitialized()) {
                if (s.options().get(Option.NEEDS_TICK).equals("true")) {
                    needsTick.add(s.options().getName());
                }
            } else {
                noErrors = false;
            }
        }
        return noErrors;
    }

    //starts all subsystems
    public void start() {
        for (SubSystem s :
                subSystems.values()) {
            s.start();
        }
        startTime();
    }

    //called to stop all subsystems
    public void stop() {
        for (SubSystem s :
                subSystems.values()) {
            s.stop();
        }
    }

    /**
     * Tick method that should be called in the tick method of {@link com.qualcomm.robotcore.eventloop.opmode.OpMode} to insure all submodules have a chance to update and get info from sensors and motors
     */
    public void tick() {
        if(needsTick !=null && !needsTick.isEmpty()) {
            for (String s :
                    needsTick) {
                SubSystem sub = subSystems.get(s);
                sub.tick();
            }
        }

    }

    public void startTime(){
        time.startTime();
    }
    public void resetTime(){
        time.reset();
    }
    public double getTimeMilliseconds(){
        return time.milliseconds();
    }
}
