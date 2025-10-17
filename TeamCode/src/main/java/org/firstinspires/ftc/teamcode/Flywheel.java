package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Flywheel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor flyWheel = hardwareMap.dcMotor.get("flyWheel");
        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // x is start, b is stop
        List<Integer> positions = new ArrayList<Integer>();
        List<Long> time = new ArrayList<>();

        double direction = 0.0;

        if (isStopRequested()) return;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            if (gamepad1.x) direction = 1.0;
            if (gamepad1.y) direction = -1.0;
            if (gamepad1.b) direction = 0.0;

            int flyWheelPosition = -flyWheel.getCurrentPosition();
            positions.add(flyWheelPosition);
            long timeCurrent = System.currentTimeMillis();
            time.add(timeCurrent);

            if (positions.size() > 10) {
                positions.remove(0);
            }
            if (time.size() > 10) {
                time.remove(0);
            }

            telemetry.addData("position", flyWheelPosition);
            telemetry.addData("direction", direction);

            int changeInPos = flyWheelPosition - positions.get(0);
            double changeInTime = timeCurrent - time.get(0);
            double velocity = ((double)changeInPos) / (changeInTime / 1000.0);
//            double power = pid.calc(FlywheelConstants.ticksInRotation, velocity);
            double error = FlywheelConstants.ticksInRotation * direction - velocity;
            double power = error * PID.Kp;

            telemetry.addData("velocity", velocity);
            telemetry.addData("delta pos", changeInPos);
            telemetry.addData("delta t", changeInTime);

            if (direction == 0.0) {
                flyWheel.setPower(0.0);
            } else {
                flyWheel.setPower(power);
            }
            telemetry.addData("power", power);


            telemetry.update();
        }
    }
}

@Config
class FlywheelConstants {
    public static double ticksInRotation = 384;
}

@Config
class PID {
    public static double Kp = 0.0001;
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
@TeleOp
class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}