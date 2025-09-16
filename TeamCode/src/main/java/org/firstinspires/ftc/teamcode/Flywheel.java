package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Flywheel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor flyWheel = hardwareMap.dcMotor.get("flyWheel");

        waitForStart();

        // x is start, b is stop

        boolean isRunning = false;

        if (isStopRequested()) return;

        int lastPosition = flyWheel.getCurrentPosition();
        ElapsedTime timer = new ElapsedTime();
        PID pid = new PID();

        while (opModeIsActive()) {
            if (gamepad1.x) isRunning = true;
            if (gamepad1.b) isRunning = false;

            telemetry.addData("position", flyWheel.getCurrentPosition());

            if (!isRunning) {
                flyWheel.setPower(0);
            } else {
                int changeInPos = flyWheel.getCurrentPosition()-lastPosition;
                double changeInTime = timer.seconds();
                double velocity = changeInPos/changeInTime;
                double power = pid.calc(FlywheelConstants.ticksInRotation, velocity);
                flyWheel.setPower(power);
                lastPosition = flyWheel.getCurrentPosition();
                telemetry.addData("velocity", velocity);
                timer.reset();
            }
            telemetry.update();
        }
    }
}

@Config
class FlywheelConstants {
    public static double ticksInRotation = 8192.0;
}

@Config
class PID {
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    double integralSum = 0;

    double lastError = 0;

    // Elapsed timer class from SDK, please use it, it's epic
    ElapsedTime timer = new ElapsedTime();

    public double calc(Double target, Double current) {
        // obtain the encoder position
        double encoderPosition = current;
        // calculate the error
        double error = target - encoderPosition;

        // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;

        // reset the timer for next time
        timer.reset();

        return out;
    }
}