/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.FTC_Library.Utilities;


/**
 * TAKEN FROM https://github.com/trc492/Ftc2017VelocityVortex/blob/master/Ftc3543Lib/src/main/java/trclib/TrcPidController.java
 * <p>
 * This class implements a PID controller. A PID controller takes a target set point and an input from a feedback
 * device to calculate the output power of an effector usually a motor or a set of motors.
 */
public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double tolerance;
    private double settlingTime;

    private boolean inverted = false;
    private boolean absSetPoint = true;
    private boolean noOscillation = false;
    private double minTarget = 0.0;
    private double maxTarget = 0.0;
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    private double prevTime = 0.0;
    private double currError = 0.0;
    private double totalError = 0.0;
    private double settlingStartTime = 0.0;
    private double setPoint = 0.0;
    private double setPointSign = 1.0;
    private double output = 0.0;


    /**
     * Constructor: Create an instance of the object.
     *
     * @param kP           specifies the proportional constant.
     * @param kI           specifies the integral constant.
     * @param kD           specifies the differential constant.
     * @param kF           specifies the feed forward constant.
     * @param tolerance    specifies the target tolerance.
     * @param settlingTime specifies the minimum on target settling time. In seconds
     */
    public PIDController(double kP, double kI, double kD, double kF, double tolerance, double settlingTime) {
        this.kP = Math.abs(kP);
        this.kI = Math.abs(kI);
        this.kD = Math.abs(kD);
        this.kF = Math.abs(kF);
        this.tolerance = Math.abs(tolerance);
        this.settlingTime = Math.abs(settlingTime);
    }

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString() {
        return PidInfo();
    }   //toString


    /**
     * This method displays the PID information on the dashboard for debugging and tuning purpose. Note that the
     * PID info occupies two dashboard lines.
     */
    public String PidInfo() {
        return "Target=" + setPoint +
                ",Error=" + currError +
                "\nminOutput=" + minOutput +
                ",Output=" + output +
                ",maxOutput=" + maxOutput;
    }   //displayPidInfo


    /**
     * This method inverts the sign of the calculated error. Normally, the calculated error starts with a large
     * positive number and goes down. However, in some sensors such as the ultrasonic sensor, the target is a small
     * number and the error starts with a negative value and increases. In order to calculate a correct output which
     * will go towards the target, the error sign must be inverted.
     *
     * @param inverted specifies true to invert the sign of the calculated error, false otherwise.
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }   //setInverted

    /**
     * This method sets the set point mode to be absolute. PID controller always calculates the output with an
     * absolute set point comparing to a sensor value representing an absolute input. But by default, it will
     * treat the set point as a value relative to its current input. So it will add the relative set point value
     * to the current input as the absolute set point in its calculation. This method allows the caller to treat
     * the set point as absolute set point.
     *
     * @param absolute specifies true if set point is absolute, false otherwise.
     */
    public void setAbsoluteSetPoint(boolean absolute) {
        absSetPoint = absolute;
    }   //setAbsoluteSetPoint

    /**
     * This method enables/disables NoOscillation mode. In PID control, if the PID constants are not tuned quite
     * correctly, it may cause oscillation that could waste a lot of time. In some scenarios, passing the target
     * beyond the tolerance may be acceptable. This method allows the PID controller to declare "On Target" even
     * though it passes the target beyond tolerance so it doesn't oscillate.
     *
     * @param noOscillation specifies true to enable no oscillation, false to disable.
     */
    public void setNoOscillation(boolean noOscillation) {
        this.noOscillation = noOscillation;
    }   //setNoOscillation

    /**
     * This method returns the current proportional constant.
     *
     * @return current proportional constant.
     */
    public double getKp() {

        return kP;
    }   //getKp

    /**
     * This method returns the current integral constant.
     *
     * @return current integral constant.
     */
    public double getKi() {
        return kI;
    }   //getKi

    /**
     * This method returns the current differential constant.
     *
     * @return current differential constant.
     */
    public double getKd() {
        return kD;
    }   //getKd

    /**
     * This method returns the current feed forward constant.
     *
     * @return current feed forward constant.
     */
    public double getKf() {

        return kF;
    }   //getKf

    /**
     * This method sets a new proportional constant.
     *
     * @param kP specifies a new proportional constant.
     */
    public void setKp(double kP) {

        this.kP = kP;
    }   //setKp

    /**
     * This method sets a new integral constant.
     *
     * @param kI specifies a new integral constant.
     */
    public void setKi(double kI) {

        this.kI = kI;
    }   //setKi

    /**
     * This method sets a new differential constant.
     *
     * @param kD specifies a new differential constant.
     */
    public void setKd(double kD) {

        this.kD = kD;
    }   //setKd

    /**
     * This method sets a new feed forward constant.
     *
     * @param kF specifies a new feed forward constant.
     */
    public void setKf(double kF) {

        this.kF = kF;
    }   //setKf

    /**
     * This method sets a new set of PID constants.
     *
     * @param kP specifies the new proportional constant.
     * @param kI specifies the new integral constant.
     * @param kD specifies the new differential constant.
     * @param kF specifies the new feed forward constant.
     */
    public void setPID(double kP, double kI, double kD, double kF) {

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }   //setPID

    /**
     * This method sets a new target tolerance.
     *
     * @param tolerance specifies the new target tolerance.
     */
    public void setTargetTolerance(double tolerance) {
        this.tolerance = tolerance;
    }   //setTargetTolerance

    /**
     * This method sets a range limit on the target set point.
     *
     * @param minTarget specifies the target set point lower range limit.
     * @param maxTarget specifies the target set point higher range limit.
     */
    public void setTargetRange(double minTarget, double maxTarget) {


        this.minTarget = minTarget;
        this.maxTarget = maxTarget;
    }   //setTargetRange

    /**
     * This method sets a range limit on the calculated output. It is very useful to limit the output range to
     * less than full power for scenarios such as using mecanum wheels on a drive train to prevent wheel slipping
     * or slow down a PID drive in order to detect a line etc.
     *
     * @param minOutput specifies the PID output lower range limit.
     * @param maxOutput specifies the PID output higher range limit.
     */
    public void setOutputRange(double minOutput, double maxOutput) {


        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }   //setOutputRange

    /**
     * This method returns the current set point value.
     *
     * @return current set point.
     */
    public double getTarget() {


        return setPoint;
    }   //getTarget

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     * @param input  the current location
     */
    public void setTarget(double target, double input) {

        if (!absSetPoint) {
            //
            // Set point is relative, add target to current input to get absolute set point.
            //
            setPoint = input + target;
            currError = target;
        } else {
            //
            // Set point is absolute, use as is.
            //
            setPoint = target;
            currError = setPoint - input;
        }

        if (inverted) {
            currError = -currError;
        }

        setPointSign = Math.signum(currError);

        //
        // If there is a valid target range, limit the set point to this range.
        //
        if (maxTarget > minTarget) {
            if (setPoint > maxTarget) {
                setPoint = maxTarget;
            } else if (setPoint < minTarget) {
                setPoint = minTarget;
            }
        }

        totalError = 0.0;
        prevTime = settlingStartTime = getCurrentTime();

    }   //setTarget

    /**
     * This method returns the error of a previous output calculation.
     *
     * @return previous error.
     */
    public double getError() {


        return currError;
    }   //getError

    /**
     * This method resets the PID controller clearing the set point, error, total error and output.
     */
    public void reset() {

        currError = 0.0;
        prevTime = 0.0;
        totalError = 0.0;
        setPoint = 0.0;
        setPointSign = 1.0;
        output = 0.0;
    }   //reset

    /**
     * This method determines if we have reached the set point target. It is considered on target if the previous
     * error is smaller than the tolerance and is maintained for at least settling time. If NoOscillation mode is
     * set, it is considered on target if we are within tolerance or pass target regardless of setting time.
     *
     * @return true if we reached target, false otherwise.
     */
    public boolean isOnTarget() {


        boolean onTarget = false;

        if (noOscillation) {
            //
            // Don't allow oscillation, so if we are within tolerance or we pass target, just quit.
            //
            if (currError * setPointSign <= tolerance) {
                onTarget = true;
            }
        } else if (Math.abs(currError) > tolerance) {
            settlingStartTime = getCurrentTime();
        } else if (getCurrentTime() >= settlingStartTime + settlingTime) {
            onTarget = true;
        }

        return onTarget;
    }   //isOnTarget

    /**
     * This method calculates the PID output applying the PID equation to the given set point target and current
     * input value.
     *
     * @return PID output value.
     */
    public double getOutput(double input) {


        double prevError = currError;
        double currTime = getCurrentTime();
        double deltaTime = currTime - prevTime;
        prevTime = currTime;
        currError = setPoint - input;
        if (inverted) {
            currError = -currError;
        }

        if (kI != 0.0) {
            //
            // Make sure the total error doesn't get wound up too much exceeding maxOutput.
            //
            double potentialGain = (totalError + currError * deltaTime) * kI;
            if (potentialGain >= maxOutput) {
                totalError = maxOutput / kI;
            } else if (potentialGain > minOutput) {
                totalError += currError * deltaTime;
            } else {
                totalError = minOutput / kI;
            }
        }

        double pTerm = kP * currError;
        double iTerm = kI * totalError;
        double dTerm = deltaTime > 0.0 ? kD * (currError - prevError) / deltaTime : 0.0;
        double fTerm = kF * setPoint;
        output = fTerm + pTerm + iTerm + dTerm;

        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < minOutput) {
            output = minOutput;
        }

        return output;
    }


    private double getCurrentTime() {
        return System.currentTimeMillis() / 1000d;//current number of system seconds. filler method that should change
    }

}