package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "two_wheel_odom_auton")
public class odom_auton extends LinearOpMode {

    // Drive motors
    DcMotorEx frontLeft, backLeft, frontRight, backRight;

    // Odometry wheels (dead wheels)
    DcMotorEx leftOdo, rightOdo;

    // ---------------- Constants ----------------
    static final double TICKS_PER_REV = 8192;
    static final double ODO_WHEEL_DIAMETER = 2.0; // inches
    static final double INCHES_PER_TICK =
            Math.PI * ODO_WHEEL_DIAMETER / TICKS_PER_REV;

    static final double DRIVE_VELOCITY = 800;

    @Override
    public void runOpMode() {

        // -------- Hardware Map --------
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        leftOdo  = hardwareMap.get(DcMotorEx.class, "leftOdo");
        rightOdo = hardwareMap.get(DcMotorEx.class, "rightOdo");

        // -------- Directions --------
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // -------- Drive Motor Setup --------
        setupDriveMotor(frontLeft);
        setupDriveMotor(backLeft);
        setupDriveMotor(frontRight);
        setupDriveMotor(backRight);

        // -------- Odometry Setup --------
        setupOdo(leftOdo);
        setupOdo(rightOdo);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        resetOdo();

        // 1️⃣ Drive straight BACK 24 inches
        driveStraightOdo(-24);

        // 2️⃣ Turn RIGHT ~45 degrees
        turnRight45();

        resetOdo();

        // 3️⃣ Drive FORWARD 36 inches
        driveStraightOdo(36);

        stopDrive();
    }

    // =====================================================
    // ================= MOVEMENT ==========================
    // =====================================================

    private void driveStraightOdo(double inches) {
        double targetTicks = inches / INCHES_PER_TICK;

        setDriveVelocity(
                Math.signum(inches) * DRIVE_VELOCITY,
                Math.signum(inches) * DRIVE_VELOCITY,
                Math.signum(inches) * DRIVE_VELOCITY,
                Math.signum(inches) * DRIVE_VELOCITY
        );

        while (opModeIsActive()) {
            double avgTicks =
                    (leftOdo.getCurrentPosition() + rightOdo.getCurrentPosition()) / 2.0;

            if (Math.abs(avgTicks) >= Math.abs(targetTicks)) break;

            telemetry.addData("Odo inches", avgTicks * INCHES_PER_TICK);
            telemetry.update();
        }

        stopDrive();
        sleep(300);
    }

    private void turnRight45() {
        // Open-loop encoder turn (approximate)
        frontLeft.setVelocity(DRIVE_VELOCITY);
        backLeft.setVelocity(DRIVE_VELOCITY);
        frontRight.setVelocity(-DRIVE_VELOCITY);
        backRight.setVelocity(-DRIVE_VELOCITY);

        // Tune this value on-field
        sleep(450);

        stopDrive();
        sleep(300);
    }

    // =====================================================
    // ================= HELPERS ===========================
    // =====================================================

    private void setDriveVelocity(double fl, double bl, double fr, double br) {
        frontLeft.setVelocity(fl);
        backLeft.setVelocity(bl);
        frontRight.setVelocity(fr);
        backRight.setVelocity(br);
    }

    private void stopDrive() {
        setDriveVelocity(0, 0, 0, 0);
    }

    private void resetOdo() {
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setupDriveMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setupOdo(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
