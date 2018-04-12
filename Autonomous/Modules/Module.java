package org.firstinspires.ftc.teamcode.FTC_API.Autonomous.Modules;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTC_API.Options;
import org.firstinspires.ftc.teamcode.FTC_API.Robot.RobotBase;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Class a module should extend in order to receive all events properly
 */

public abstract class Module {
    protected RobotBase robot;
    protected int positionInArray;
    private int internalPositionInArray;//used internally, should only be used by this class to pass though array position by default

    protected Telemetry telemetry;

    /**
     * @param robot           the robot to reference
     * @param positionInArray where this module was stored in array (position 0, position 1, position 2, etc.)
     */
    public void init(RobotBase robot, final int positionInArray, Telemetry telemetry) {
        this.robot = robot;
        this.positionInArray = positionInArray;
        internalPositionInArray = positionInArray;//set internal
        this.telemetry = telemetry;
    }

    abstract public void start();

    abstract public void tick();

    //NOTE: This expects an int between 0 and the number of possible next steps - 1
    //This ideally should not rely on constants however there really is no way around this

    /**
     * Default behavior is to pass though position in array to the next step
     *
     * @return
     */
    public int stop() {
        return internalPositionInArray;//return default position in array, unless it has been overridden by resetPositionInArray()
    }

    /**
     * Call this LAST in constructor chain else you won't have access to module specific functions
     * Used to reset the next running position in array to 0
     *
     * @return object for building
     */
    public Module resetArrayPosition() {
        internalPositionInArray = 0;
        return this;
    }

    /**
     * Call this LAST in constructor chain else you won't have access to module specific functions
     * Used to set next running position in array
     *
     * @return object for building
     */
    public Module setArrayPosition(int position) {
        internalPositionInArray = position;
        return this;
    }

    abstract public Options options();

    public String ID() {
        return options().getName();
    }

    protected boolean hasTelemetry() {
        return telemetry != null;
    }

    abstract public boolean isDone();

    abstract public String[] requiredSubSystems();
}
