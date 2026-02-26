package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivePID {

    private DcMotorEx fl, fr, bl, br;
    private BNO055IMU imu;

    private PIDController distancePID;
    private PIDController headingPID;

    private double ticksPerInch;

    // Constructor
    public DrivePID(HardwareMap hardwareMap,
                    double distP, double distI, double distD,
                    double headP, double headI, double headD,
                    double ticksPerInch) {

        fl = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        fr = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        bl = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        br = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        distancePID = new PIDController(distP, distI, distD);
        headingPID = new PIDController(headP, headI, headD);

        this.ticksPerInch = ticksPerInch;

        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters p = new BNO055IMU.Parameters();
            p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu.initialize(p);
        } catch (Exception e) {
            imu = null;
        }
    }

    public void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveToDistance(double inches,
                                double maxPower,
                                double minPower,
                                int posTolerance,
                                double velTolerance) {

        resetEncoders();

        int targetTicks = (int)(inches * ticksPerInch);

        while (Math.abs(targetTicks - getAveragePosition()) > posTolerance) {

            int pos = getAveragePosition();
            int error = targetTicks - pos;

            double forward = distancePID.calculate(pos, targetTicks);
            forward = clamp(forward, -maxPower, maxPower);

            if (Math.abs(error) > posTolerance) {
                forward = applyMinPower(forward, minPower);
            }

            setDrivePower(forward, 0);
        }

        stop();
    }

    private int getAveragePosition() {
        return (fl.getCurrentPosition() +
                fr.getCurrentPosition() +
                bl.getCurrentPosition() +
                br.getCurrentPosition()) / 4;
    }

    private void setDrivePower(double forward, double turn) {
        double left = forward - turn;
        double right = forward + turn;

        fl.setPower(left);
        bl.setPower(left);
        fr.setPower(right);
        br.setPower(right);
    }

    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double applyMinPower(double power, double minAbs) {
        if (power == 0) return 0;
        double sign = Math.signum(power);
        return sign * Math.max(Math.abs(power), minAbs);
    }
}