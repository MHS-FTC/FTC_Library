package org.firstinspires.ftc.teamcode.FTC_API.Robot.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class SidedDriveSystemTemplate extends DriveSystemTemplate {
    public abstract DcMotor[] getRightSideMotors();

    public abstract DcMotor[] getLeftSideMotors();

    @Override
    public DcMotor[] getMotors() {
        return combine(getLeftSideMotors(),getRightSideMotors());
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

     private static DcMotor[] combine(DcMotor[] a, DcMotor[] b){
        int length = a.length + b.length;
        DcMotor[] result = new DcMotor[length];
        System.arraycopy(a, 0, result, 0, a.length);
        System.arraycopy(b, 0, result, a.length, b.length);
        return result;
    }
}
