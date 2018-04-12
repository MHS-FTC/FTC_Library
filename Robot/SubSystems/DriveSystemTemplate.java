package org.firstinspires.ftc.teamcode.FTC_API.Robot.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

/**
 * Created by ethan.hampton on 12/15/2017.
 * <p>
 * The template that all drive systems should use so they can easily be controlled by autonomous.
 */

public abstract class DriveSystemTemplate extends SubSystem {
    public abstract DcMotor[] getRightSideMotors();

    public abstract DcMotor[] getLeftSideMotors();

    public abstract void driveTank(double leftPower, double rightPower);

    /**
     * @param forward amount forward from -1 to 1 inclusive
     * @param turn    amount to turn from -1 to 1 inclusive, -1 being full power left, 1 being full power right
     */
    public abstract void driveArcade(double forward, double turn);

    public abstract void driveMecanum(double forward, double turn, double strafe);

    public MotorConfigurationType getMotorType() {
        return getRightSideMotors()[0].getMotorType();
    }

    public void setLeftSideTarget(int ticks) {
        for (DcMotor m : getLeftSideMotors()) {
            m.setTargetPosition(ticks);
        }
    }

    public void setRightSideTarget(int ticks) {
        for (DcMotor m : getRightSideMotors()) {
            m.setTargetPosition(ticks);
        }
    }

    public void resetAllEncoders() {
        for (DcMotor m : getLeftSideMotors()) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for (DcMotor m : getRightSideMotors()) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void runToPositionAllEncoders() {
        for (DcMotor m : getLeftSideMotors()) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (DcMotor m : getRightSideMotors()) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void runUsingAllEncoders() {
        for (DcMotor m : getLeftSideMotors()) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        for (DcMotor m : getRightSideMotors()) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void runNotUsingAllEncoders() {
        for (DcMotor m : getLeftSideMotors()) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (DcMotor m : getRightSideMotors()) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void stopRightMotors() {
        for (DcMotor m :
                getRightSideMotors()) {
            m.setPower(0);
        }
    }

    public void stopLeftMotors() {
        for (DcMotor m :
                getLeftSideMotors()) {
            m.setPower(0);
        }
    }

    public void stopAllMotors() {
        stopLeftMotors();
        stopRightMotors();
    }
}
