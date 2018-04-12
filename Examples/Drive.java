package org.firstinspires.ftc.teamcode.FTC_API.Examples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.FTC_API.Options;
import org.firstinspires.ftc.teamcode.FTC_API.Robot.SubSystems.DriveSystemTemplate;

/**
 * Created by Ethan Hampton on 8/19/17.
 * <p>
 * Simple Drive class that can be implemented and used
 */

public class Drive extends DriveSystemTemplate {
    private Options options = new Options(ID);
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;

    public static final String ID = "Drive";

    @Override
    public boolean init(HardwareMap hardwareDevices) {
        leftMotor = hardwareDevices.dcMotor.get(options.get("leftMotor"));
        rightMotor = hardwareDevices.dcMotor.get(options.get("rightMotor"));
        return true;
    }

    public Drive setMotorNames(String left, String right) {
        options.add("leftMotor", left);
        options.add("rightMotor", right);
        return this;
    }

    public Drive setMotorType(MotorConfigurationType type) {
        leftMotor.setMotorType(type);
        rightMotor.setMotorType(type);
        return this;
    }

    public void drive(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    @Override
    public Options options() {
        return options;
    }

    @Override
    public DcMotor[] getRightSideMotors() {
        return new DcMotor[]{rightMotor};
    }

    @Override
    public DcMotor[] getLeftSideMotors() {
        return new DcMotor[]{leftMotor};
    }

    @Override
    public void driveTank(double leftPower, double rightPower) {
        drive(leftPower, rightPower);
    }

    @Override
    public void driveArcade(double forward, double turn) {
        double left = forward + turn;
        double right = forward - turn;

        driveTank(left, right);
    }

    @Override
    public void driveMecanum(double forward, double turn, double strafe) {
        driveArcade(forward, turn);
    }
}
