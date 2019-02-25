package org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTC_Library.Robot.RobotBase;

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
     * Initializes the module with the information it needs
     * @param robot           the robot to reference
     * @param positionInArray where this module was stored in array (position 0, position 1, position 2, etc.)
     */
    public void init(RobotBase robot, final int positionInArray, Telemetry telemetry) {
        this.robot = robot;
        this.positionInArray = positionInArray;
        internalPositionInArray = positionInArray;//set internal
        this.telemetry = telemetry;
    }

    /**
     * Called at start of run
     */
    abstract public void start();

    /**
     * Called every tick
     */
    abstract public void tick();

    //

    /**
     * Default behavior is to pass though position in array to the next step
     * <p>
     * NOTE: This expects an int between 0 and the number of possible next steps - 1
     * This ideally should not rely on constants however there really is no way around this
     *
     * @return The next position in the array to execute
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

    @Deprecated
    public String ID() {
        return "";
    }

    /**
     * @return if telemetry has been set up
     * @deprecated This will now always return true
     */
    @Deprecated
    protected boolean hasTelemetry() {
        return telemetry != null;
    }

    /**
     * This is called each tick and should return true if this module is done executing
     *
     * @return true if module is done executing
     */
    abstract public boolean isDone();

    @Deprecated
    public String[] requiredSubSystems() {
        return new String[0];
    }
}
