package org.firstinspires.ftc.teamcode.FTC_API.Autonomous.Modules;

import org.firstinspires.ftc.teamcode.FTC_API.Options;
import org.firstinspires.ftc.teamcode.FTC_API.Robot.Robot;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Class a module should extend in order to receive all events properly
 */

public abstract class Module {
    protected Robot robot;

    public void init(Robot robot) {
        this.robot = robot;
    }

    abstract public void start();

    abstract public void tick();

    //NOTE: This expects an int between 0 and the number of possible next steps - 1
    //This ideally should not rely on constants however there really is no way around this
    public int stop() {
        return 0;
    }

    abstract public Options options();

    public String ID() {
        return options().getName();
    }

    abstract public boolean isDone();

    abstract public String[] requiredSubSystems();
}
