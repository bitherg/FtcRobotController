package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PID Distance Auto (FTCLib) - Tuning", group = "Auto")
public class PIDDistanceAutoFTCLib extends LinearOpMode {

    // ======= Hardware =======
    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // Optional IMU heading hold (recommended if you have an IMU)
    private BNO055IMU imu;
    private boolean useHeadingHold = false;

    // =======================
    // ===== PID TUNING ======
    // =======================

    // ---- Toggle tuning mode ----
    private static final boolean TUNING_MODE = true;

    // ---- Test distance (change this while tuning) ----
    private static final double TEST_DISTANCE = 24.0;   // try 6, 24, 36

    // ---- Distance PID ----
    private static double DIST_P = 0.012;
    private static double DIST_I = 0.0;
    private static double DIST_D = 0.0008;

    // ---- Heading PID ----
    private static double HEAD_P = 0.015;
    private static double HEAD_I = 0.0;
    private static double HEAD_D = 0.0006;

    // ---- Motion limits ----
    private static double MAX_POWER = 0.65;
    private static double MIN_POWER = 0.08;

    // ---- Stop conditions ----
    private static int POS_TOL_TICKS = 40;
    private static double VEL_TOL_TPS = 60;

    // Controllers (created in initHardware)
    private PIDController distancePID;
    private PIDController headingPID;

    // ======= Safety timeout =======
    private static final double TIMEOUT_S = 4.0;      // safety timeout for the move

    // ======= Encoder / Robot constants =======
    // Set this to YOUR motor ticks per rev.
    // Common examples:
    // - goBILDA 5202/5203 312RPM: 537.7 ticks/rev
    // - goBILDA 5202/5203 435RPM: 383.6 ticks/rev
    private static final double TICKS_PER_REV = 537.7;

    // Wheel + gear ratio
    private static final double WHEEL_DIAMETER_IN = 3.78;  // change to yours
    private static final double GEAR_RATIO = 1.0;          // output/input, usually 1.0 unless geared

    // Optional: use this to correct measured distance (ex: 1.03 if it goes short)
    private static final double DISTANCE_SCALE = 1.00;

    private static final double TICKS_PER_INCH =
            DISTANCE_SCALE * (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Ready. Press start.");
        telemetry.addLine(TUNING_MODE ? "TUNING_MODE = ON" : "TUNING_MODE = OFF");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Use current heading as the heading to hold (only if IMU is present)
        double holdHeadingDeg = useHeadingHold ? getHeadingDeg() : 0.0;

        if (TUNING_MODE) {
            driveToDistancePID(TEST_DISTANCE, holdHeadingDeg);
        } else {
            driveToDistancePID(24.0, holdHeadingDeg);
        }

        stopAll();
    }

    private void initHardware() {
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Reverse as needed (common: reverse left side)
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Brake helps stop closer to target
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetAndRunEncoders();

        // IMU (optional)
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters p = new BNO055IMU.Parameters();
            p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu.initialize(p);
            useHeadingHold = true;
        } catch (Exception e) {
            useHeadingHold = false;
        }

        // PID controllers from tuning variables
        distancePID = new PIDController(DIST_P, DIST_I, DIST_D);
        headingPID  = new PIDController(HEAD_P, HEAD_I, HEAD_D);

        distancePID.setTolerance(POS_TOL_TICKS);
        headingPID.setTolerance(1.5); // degrees
    }

    /**
     * Drive forward/backward a given number of inches using PID on average encoder position.
     * @param inches positive forward, negative backward
     * @param holdHeadingDeg heading to maintain (ignored if no IMU)
     */
    private void driveToDistancePID(double inches, double holdHeadingDeg) {
        resetAndRunEncoders();

        int targetTicks = (int) Math.round(inches * TICKS_PER_INCH);

        distancePID.reset();
        headingPID.reset();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < TIMEOUT_S) {
            int pos = getAverageEncoderTicks();
            int error = targetTicks - pos;

            // Distance PID output: forward power
            double forward = distancePID.calculate(pos, targetTicks);

            // Clamp and add minimum power to overcome stiction
            forward = clamp(forward, -MAX_POWER, MAX_POWER);
            if (Math.abs(error) > POS_TOL_TICKS) {
                forward = applyMinPower(forward, MIN_POWER);
            }

            // Heading correction (turn) if IMU present
            double turn = 0.0;
            double heading = 0.0;
            double headingError = 0.0;

            if (useHeadingHold) {
                heading = getHeadingDeg();
                headingError = angleWrapDeg(holdHeadingDeg - heading);
                turn = headingPID.calculate(heading, holdHeadingDeg);
                turn = clamp(turn, -0.25, 0.25);
            }

            // Apply to drivetrain (arcade, no strafe)
            setDrivePowers(forward, turn);

            // Stop conditions: within position tolerance AND slowed down
            double vel = getAverageVelocityTicksPerSec();
            boolean atPos = Math.abs(error) <= POS_TOL_TICKS;
            boolean settled = Math.abs(vel) <= VEL_TOL_TPS;

            // ====== TUNING TELEMETRY ======
            telemetry.addLine("===== PID TUNING =====");
            telemetry.addData("TestInches", inches);
            telemetry.addData("TICKS_PER_INCH", "%.3f", TICKS_PER_INCH);

            telemetry.addData("DIST P", DIST_P);
            telemetry.addData("DIST I", DIST_I);
            telemetry.addData("DIST D", DIST_D);

            telemetry.addData("HEAD P", HEAD_P);
            telemetry.addData("HEAD I", HEAD_I);
            telemetry.addData("HEAD D", HEAD_D);

            telemetry.addData("MAX_POWER", MAX_POWER);
            telemetry.addData("MIN_POWER", MIN_POWER);

            telemetry.addData("POS_TOL_TICKS", POS_TOL_TICKS);
            telemetry.addData("VEL_TOL_TPS", VEL_TOL_TPS);

            telemetry.addLine("===== RUN DATA =====");
            telemetry.addData("TargetTicks", targetTicks);
            telemetry.addData("Pos", pos);
            telemetry.addData("Error", error);
            telemetry.addData("Forward", forward);
            telemetry.addData("Turn", turn);
            telemetry.addData("Vel(t/s)", "%.1f", vel);
            telemetry.addData("AtPos", atPos);
            telemetry.addData("Settled", settled);
            telemetry.addData("Time", "%.2f", timer.seconds());

            if (useHeadingHold) {
                telemetry.addData("Heading", "%.1f", heading);
                telemetry.addData("HeadingErr", "%.1f", headingError);
            } else {
                telemetry.addLine("IMU: not detected (heading hold OFF)");
            }

            telemetry.update();

            if (atPos && settled) break;
        }

        stopAll();
        sleep(150);
    }

    // ======= Helpers =======

    private void resetAndRunEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private int getAverageEncoderTicks() {
        return (frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition()
                + backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4;
    }

    private double getAverageVelocityTicksPerSec() {
        // DcMotorEx provides getVelocity() in ticks/second
        return (frontLeftMotor.getVelocity() + frontRightMotor.getVelocity()
                + backLeftMotor.getVelocity() + backRightMotor.getVelocity()) / 4.0;
    }

    /**
     * forward = + drives forward. turn = + turns right.
     * Works for tank or mecanum as "arcade" on all 4 motors (no strafe).
     */
    private void setDrivePowers(double forward, double turn) {
        double left = forward - turn;
        double right = forward + turn;

        // normalize if needed
        double max = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        left /= max;
        right /= max;

        frontLeftMotor.setPower(left);
        backLeftMotor.setPower(left);
        frontRightMotor.setPower(right);
        backRightMotor.setPower(right);
    }

    private void stopAll() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double applyMinPower(double power, double minAbs) {
        if (power == 0) return 0;
        double sign = Math.signum(power);
        return sign * Math.max(Math.abs(power), minAbs);
    }

    private double getHeadingDeg() {
        // BNO055IMU: firstAngle is typically Z (yaw) for FTC default orientation
        return imu.getAngularOrientation().firstAngle;
    }

    private static double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}