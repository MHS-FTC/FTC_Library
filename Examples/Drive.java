package org.firstinspires.ftc.teamcode.FTC_API.Examples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.FTC_API.Options;
import org.firstinspires.ftc.teamcode.FTC_API.Robot.SubSystems.SubSystem;

/**
 * Created by Ethan Hampton on 8/19/17.
 *
 * Simple Drive class that can be implemented and used
 */

public class Drive extends SubSystem {
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

    public Drive setMotorNames(String left, String right){
        options.add("leftMotor", left);
        options.add("rightMotor", right);
        return this;
    }

    public Drive setMotorType(MotorConfigurationType type){
        leftMotor.setMotorType(type);
        rightMotor.setMotorType(type);
        return this;
    }

    public void drive(double leftPower, double rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    @Override
    public Options options() {
        return options;
    }

}
