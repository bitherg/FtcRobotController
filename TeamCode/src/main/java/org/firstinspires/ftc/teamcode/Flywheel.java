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
@TeleOp(name = "Flywheel_SPLITSTICK_FIXED_FINAL")
public class Flywheel extends LinearOpMode {

    // Blocker CRServo
    double blockerPower = 1.0;
    double blockerSpinTime = 1.5;

    // Turning coefficient for proportional turning
    static double turnCoefficient = 0.3;

    @Override
    public void runOpMode() {

        // ================= TIMERS =================
        boolean blockerRunning = false;
        ElapsedTime blockerTimer = new ElapsedTime();

        // ================= DRIVETRAIN =================
        DcMotor frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor  = hardwareMap.dcMotor.get("backRightMotor");

        // ✅ Motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); // only back left reversed
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // ================= FLYWHEELS =================
        DcMotorEx flyWheel   = hardwareMap.get(DcMotorEx.class, "flyWheel");
        DcMotorEx flyWheel_2 = hardwareMap.get(DcMotorEx.class, "flyWheel_2");
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ================= INTAKE =================
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ================= BLOCKER =================
        CRServo blocker = hardwareMap.get(CRServo.class, "blocker");
        blocker.setPower(0.0);

        telemetry.addLine("TeleOp Ready");
        telemetry.update();

        waitForStart();

        double flyspeed = 1.0;
        double flywheelDirection = 0.0;
        boolean wasPressedUp = false;

        while (opModeIsActive()) {

            // ================= BLOCKER CONTROL =================
            if (gamepad1.yWasReleased() && !blockerRunning) {
                blocker.setPower(-blockerPower); // reversed to hit balls
                blockerTimer.reset();
                blockerRunning = true;
            }
            if (blockerRunning && blockerTimer.seconds() >= blockerSpinTime) {
                blocker.setPower(0.0);
                blockerRunning = false;
            }

            // ================= FLYWHEEL CONTROL =================
            if (gamepad1.right_trigger > 0.6) flywheelDirection = 1.0;
            if (gamepad1.left_trigger > 0.6)  flywheelDirection = -1.0;
            if (gamepad1.a) flywheelDirection = 0.0;

            if (gamepad1.dpad_up && flyspeed < 1.0 && !wasPressedUp) flyspeed += 0.2;
            wasPressedUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && flyspeed > 0.2) flyspeed -= 0.2;

            flyWheel.setPower(-flywheelDirection * flyspeed);
            flyWheel_2.setPower(-flywheelDirection * flyspeed);

            // ================= INTAKE =================
            if (gamepad1.right_bumper) intake.setPower(-1.0);
            else if (gamepad1.left_bumper) intake.setPower(1.0);
            else if (gamepad1.b) intake.setPower(0.0);

            // ================= MECANUM DRIVE =================
            double leftStickY = -gamepad1.left_stick_y;  // invert: up = positive
            double leftStickX = gamepad1.left_stick_x;   // turning bias
            double rightStickX = gamepad1.right_stick_x; // strafe

            // Deadzone
            if (Math.abs(leftStickY) < 0.05) leftStickY = 0;
            if (Math.abs(leftStickX) < 0.05) leftStickX = 0;
            if (Math.abs(rightStickX) < 0.05) rightStickX = 0;

            // Proportional turning only when moving forward/back
            double turnBias = 0;
            if (Math.abs(leftStickY) > 0.05) {
                turnBias = leftStickX * Math.abs(leftStickY) * turnCoefficient;
            }

            // Corrected mecanum formula for your motor directions
            double fl = leftStickY + rightStickX + turnBias;
            double bl = leftStickY + rightStickX - turnBias;  // back left reversed motor
            double fr = leftStickY - rightStickX - turnBias;
            double br = leftStickY - rightStickX + turnBias;

            // Normalize
            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftMotor.setPower((y + x + rx) / denominator);
            backLeftMotor.setPower((y - x + rx) / denominator);
            frontRightMotor.setPower((y - x - rx) / denominator);
            backRightMotor.setPower((y + x - rx) / denominator);

            telemetry.addData("Flywheel Dir (-1 is shooting)", flywheelDirection);
            telemetry.addData("Intake dir", intake.getPower());
            telemetry.addData("Blocker Down", blockerRunning);
            telemetry.addData("fly speed", flyspeed);
            telemetry.update();
        }
    }
}
