package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class Flywheel extends LinearOpMode {

    // ===== Blocker (CR Servo) =====
    double blockerPower = 1.0;        // spin direction / speed
    double blockerSpinTime = 2.0;     // seconds

    @Override
    public void runOpMode() throws InterruptedException {

        // Blocker timing
        boolean blockerRunning = false;
        ElapsedTime blockerTimer = new ElapsedTime();

        // Drivetrain motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flywheel motors
        DcMotorEx flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        DcMotorEx flyWheel_2 = hardwareMap.get(DcMotorEx.class, "flyWheel_2");

        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flyWheel_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Intake
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== CONTINUOUS ROTATION SERVO =====
        CRServo blocker = hardwareMap.get(CRServo.class, "blocker");

        waitForStart();

        boolean wasPressedup = false;
        double flyspeed = 1.0;
        double flywheelDirection = 0;

        while (opModeIsActive()) {

            // ================= BLOCKER CONTROL =================

            // Press Y → spin blocker
            if (gamepad1.yWasReleased() && !blockerRunning) {
                blocker.setPower(blockerPower);   // start spinning
                blockerTimer.reset();
                blockerRunning = true;
            }

            // Stop after set time
            if (blockerRunning && blockerTimer.seconds() >= blockerSpinTime) {
                blocker.setPower(0.0);            // stop
                blockerRunning = false;
            }

            // ================= FLYWHEEL CONTROL =================

            if (gamepad1.right_trigger > 0.6) flywheelDirection = 1.0;
            if (gamepad1.left_trigger > 0.6) flywheelDirection = -1.0;
            if (gamepad1.dpad_up && flyspeed < 1 && !wasPressedup) {
                flyspeed += 0.2;
            }
            wasPressedup = gamepad1.dpad_up;

            if (gamepad1.dpad_down && flyspeed > 1e-5) flyspeed -= 0.2;
            if (gamepad1.a) flywheelDirection = 0.0;

            flyWheel.setPower(-flywheelDirection * flyspeed);
            flyWheel_2.setPower(-flywheelDirection * flyspeed);

            // ================= INTAKE CONTROL =================

            if (gamepad1.right_bumper) intake.setPower(-1);
            else if (gamepad1.left_bumper) intake.setPower(1);
            else if (gamepad1.b) intake.setPower(0.0);

            // ================= MECANUM DRIVE =================

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_y;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftMotor.setPower((y + x + rx) / denominator);
            backLeftMotor.setPower((y - x + rx) / denominator);
            frontRightMotor.setPower((y - x - rx) / denominator);
            backRightMotor.setPower((y + x - rx) / denominator);

            // ================= TELEMETRY =================

            telemetry.addData("Blocker Running", blockerRunning);
            telemetry.addData("Blocker Time", blockerTimer.seconds());
            telemetry.addData("Flywheel Dir", flywheelDirection);
            telemetry.addData("Fly Speed", flyspeed);
            telemetry.update();
        }
    }
}
