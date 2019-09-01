package org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Premade;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTC_Library.Autonomous.Modules.Module;

public class Gyroscope extends Module {
    private static boolean hasCalibrated = false;
    private static GyroSensor gyro;

    private int targetDegrees = 0;
    private boolean calibrate = false;

    private final double midPower = 0.3;
    private final double maxPower = 0.85;
    private final int MAX_ERROR = 3;//degrees

    @Override
    public void start() {
        gyro.resetZAxisIntegrator();
        if (calibrate || !hasCalibrated) {
            telemetry.addLine("Calibrating Gyro...");
            gyro.calibrate();
            while (gyro.isCalibrating()) {
                //wait for calibration to finish
                telemetry.addLine("Calibrating Gyro...");
            }
            hasCalibrated = true;
        }
    }

    @Override
    public boolean tick() {
        int currentHeading = gyro.getHeading();
        if (currentHeading > 180) {
            currentHeading = currentHeading - 360;
        }
        int headingError = targetDegrees - currentHeading;
        double driveSteering = headingError * 0.005;

        telemetry.addLine("Current Heading = " + currentHeading);
        telemetry.addLine("Heading Error: " + headingError);
        telemetry.addLine("Steering Error: " + driveSteering);

        double leftPower, rightPower;
        if (headingError < 0) {
            leftPower = -midPower + driveSteering;
            rightPower = midPower - driveSteering;
        } else {
            leftPower = midPower + driveSteering;
            rightPower = -midPower - driveSteering;
        }

        leftPower = Range.clip(leftPower, -maxPower, maxPower);
        rightPower = Range.clip(rightPower, -maxPower, maxPower);

        robot.getDriveSystem().driveTank(leftPower, rightPower);

        return Math.abs(headingError) < MAX_ERROR;//return done if the heading error is less than the max allowable error
    }

    public int stop() {
        robot.getDriveSystem().driveTank(0, 0);//Stop motors before continuing
        return positionInArray;
    }

    public Gyroscope setTurn(int degrees) {
        targetDegrees = -degrees;
        return this;
    }

    public Gyroscope setCalibrate(boolean cal) {
        calibrate = cal;
        return this;
    }

    public static void setGyro(GyroSensor gyroSensor){
        gyro = gyroSensor;
    }
}
