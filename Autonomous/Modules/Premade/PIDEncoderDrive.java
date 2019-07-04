package org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Premade;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Module;
import org.firstinspires.ftc.teamcode.FTC_Library.Robot.SubSystems.SidedDriveSystemTemplate;
import org.firstinspires.ftc.teamcode.FTC_Library.Utilities.PIDController;

/**
 * PIDEncoderDrive is designed to make autonomous driving really easy for most teams but also extendable as teams get more advanced.
 * It has been known for teams to spend multiple hours tuning the PID values of their robot to get it exactly right but also at the same time,
 * spending 15 minutes can be more than enough to get acceptable results.
 * <p>
 * Ideally your main robot class will have a PIDEncoderDrive.PIDConfig that you can use for all your autonomous movement.
 * This way instead of having to set up each PIDEncoderDrive, you just have to give it the config.
 * Multiple configs for one program are also possible...
 *
 */
public class PIDEncoderDrive extends Module {
    private SidedDriveSystemTemplate drive;
    private double startTime;//TODO add watchdog

    private double leftRotations;
    private double rightRotations;

    private double leftSpeed;
    private double rightSpeed;
    private PIDController leftController;
    private PIDController rightController;
    private double wheelCircumference;

    /**
     * -Gets the drive system
     * -sets each side's target encoder ticks for motors
     * -resets encoders
     * -zeros out speeds as needed
     */
    @Override
    public void start() {
        if (robot.getDriveSystem() instanceof SidedDriveSystemTemplate) {
            drive = (SidedDriveSystemTemplate) robot.getDriveSystem();
        } else {
            //if not a sided drive system, then exit set up
            return;
        }

        startTime = robot.getTimeMilliseconds();

        //sets targets
        long leftTarget = -(int) (leftRotations * drive.getMotorType().getTicksPerRev());
        long rightTarget = -(int) (rightRotations * drive.getMotorType().getTicksPerRev());

        drive.resetAllEncoders();
        drive.runUsingAllEncoders();


        leftController.setTarget(leftTarget, 0);//set PID controllers
        rightController.setTarget(rightTarget, 0);

        //if either motor doesn't need to move then don't move it
        if (leftTarget == 0) {
            leftSpeed = 0;
        }
        if (rightTarget == 0) {
            rightSpeed = 0;
        }

        drive.driveTank(leftSpeed, rightSpeed);//currently the speeds are 0 but I don't think it does any harm leaving it this way

        if (hasTelemetry()) {
            telemetry.log().add("Right Target:" + rightTarget + " Left Target:" + leftTarget);
            telemetry.log().add("Right Rotations:" + rightRotations + " Left Rotations:" + leftRotations);
        }
    }

    /**
     * This function runs:
     * -some fail safe code
     * -averages the motor encoder positions on each side of the robot
     * -feeds that into the PID controller
     * -gets the recommended output and applies that to the motors
     * -stops when both sides are on target
     * @return if this module is done
     */
    @Override
    public boolean tick() {
        //some fail safes
        if (drive == null) {
            //if not a sided drive system, then exit
            telemetry.log().add("Need SidedDriveSystem");
            return true;
        }
        if (wheelCircumference == 0) {
            telemetry.log().add("Please add wheel circumference");
            return true;
        }

        int currentLeft = 0;
        int motorsLeft = 0;
        int currentRight = 0;
        int motorsRight = 0;

        for (DcMotor motor : drive.getLeftSideMotors()) {
            currentLeft += motor.getCurrentPosition();
            motorsLeft++;
        }
        for (DcMotor motor : drive.getRightSideMotors()) {
            currentRight += motor.getCurrentPosition();
            motorsRight++;
        }

        if (motorsRight == 0 || motorsLeft == 0) {
            //need at least one motor per side
            telemetry.log().add("One or both sides of the robot don't have motors!");
            return true;
        }

        //update speed so as to not overshoot
        leftSpeed = leftController.getOutput(currentLeft / motorsLeft);
        rightSpeed = rightController.getOutput(currentRight / motorsRight);
        drive.driveTank(leftSpeed, rightSpeed);

        telemetry.addLine("Left Speed:" + leftSpeed + ", Right Speed:" + rightSpeed);


        //stop if on target, and end if both are on target
        if (leftController.isOnTarget()) {
            drive.stopLeftMotors();
        }
        if (rightController.isOnTarget()) {
            drive.stopRightMotors();
        }
        //when both are on target, this system is done
        return leftController.isOnTarget() && rightController.isOnTarget();
    }

    @Override
    public int stop() {
        drive.runUsingAllEncoders();
        //just pass through the position
        return positionInArray;
    }

    /**
     * Resets the position in array number so you can changing it in the next step
     *
     * @return this object for building
     */
    public PIDEncoderDrive resetPositionInArray() {
        positionInArray = 0;
        return this;
    }


    public PIDEncoderDrive setPID(double p, double i, double d, double settlingTime, int tolerance, double maxSpeed) {
        leftController = new PIDController(p, i, d, 0, tolerance, settlingTime);
        rightController = new PIDController(p, i, d, 0, tolerance, settlingTime);

        leftController.setNoOscillation(true);//we don't want to change direction to correct to target
        rightController.setNoOscillation(true);

        leftController.setOutputRange(-maxSpeed, maxSpeed);
        rightController.setOutputRange(-maxSpeed, maxSpeed);
        return this;
    }

    /**
     * @param distanceLeft  distance in inches the left motor(s) should go
     * @param distanceRight distance in inches the right motor(s) should go
     * @return this object for building
     */
    public PIDEncoderDrive setDistances(double distanceLeft, double distanceRight) {
        leftRotations = distanceLeft / wheelCircumference;
        rightRotations = distanceRight / wheelCircumference;
        return this;
    }

    public PIDEncoderDrive setWheelCircumference(double circumference) {
        wheelCircumference = circumference;
        return this;
    }

    public PIDEncoderDrive setConfig(PIDConfig config) {
        setWheelCircumference(config.wheelCircumference);
        setPID(config.p, config.i, config.d, config.settlingTime, config.tolerance, config.maxSpeed);
        return this;
    }

    public static class PIDConfig {
        private double p, i, d, wheelCircumference, settlingTime;
        private int tolerance;
        private double maxSpeed;

        /**
         *
         * @param p the proportional value in PID
         * @param i the integral value in PID
         * @param d the derivative value in PID
         * @return object for building
         */
        public PIDConfig setPID(double p, double i, double d) {
            this.p = p;
            this.i = i;
            this.d = d;
            return this;
        }

        public double getP() {
            return p;
        }

        public PIDConfig setP(double p) {
            this.p = p;
            return this;
        }

        public double getI() {
            return i;
        }

        public PIDConfig setI(double i) {
            this.i = i;
            return this;
        }

        public double getD() {
            return d;
        }

        public PIDConfig setD(double d) {
            this.d = d;
            return this;
        }

        public double getWheelCircumference() {
            return wheelCircumference;
        }

        /**
         *
         * @param wheelCircumference The circumference of your wheels. It is okay to say diameter * 3.141, that is plenty of precision
         * @return object for building
         */
        public PIDConfig setWheelCircumference(double wheelCircumference) {
            this.wheelCircumference = wheelCircumference;
            return this;
        }

        public double getSettlingTime() {
            return settlingTime;
        }

        /**
         *
         * @param settlingTime how long the robot needs to be within tolerance of the specified position before being considered done moving
         * @return object for building
         */
        public PIDConfig setSettlingTime(double settlingTime) {
            this.settlingTime = settlingTime;
            return this;
        }

        public int getTolerance() {
            return tolerance;
        }

        /**
         *
         * @param tolerance how many encoder ticks away from the target value is considered 'close enough'
         * @return object for building
         */
        public PIDConfig setTolerance(int tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public double getMaxSpeed() {
            return maxSpeed;
        }

        /**
         * Note that the PID drive may never reach this speed depending on distance and what PID values are given
         * @param maxSpeed max speed you want to let your robot go
         * @return object for building
         */
        public PIDConfig setMaxSpeed(double maxSpeed) {
            this.maxSpeed = maxSpeed;
            return this;
        }
    }
}