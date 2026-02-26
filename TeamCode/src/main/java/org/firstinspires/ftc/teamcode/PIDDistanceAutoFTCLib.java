package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// FTC Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
@Autonomous(name = "PID Distance Auto (FTCLib) - Dashboard Tuning", group = "Auto")
public class PIDDistanceAutoFTCLib extends LinearOpMode {

    // ======= Hardware =======
    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // Optional IMU heading hold
    private BNO055IMU imu;
    private boolean imuDetected = false;

    // =======================
    // ===== DASHBOARD TUNING =
    // =======================
    // NOTE: Dashboard only edits public static fields

    // ---- Toggle tuning mode ----
    public static boolean TUNING_MODE = true;

    // ---- Test distance (change live) ----
    public static double TEST_DISTANCE = 24.0;   // try 6, 24, 36

    // ---- Distance PID (change live) ----
    public static double DIST_P = 0.012;
    public static double DIST_I = 0.0;
    public static double DIST_D = 0.0008;

    // ---- Heading Hold ----
    public static boolean USE_HEADING_HOLD = false; // turn on after distance is tuned
    public static double HEAD_P = 0.015;
    public static double HEAD_I = 0.0;
    public static double HEAD_D = 0.0006;
    public static double TURN_CLAMP = 0.25; // max turn power from heading hold

    // ---- Motion limits ----
    public static double MAX_POWER = 0.65;
    public static double MIN_POWER = 0.08;

    // ---- Stop conditions ----
    public static int POS_TOL_TICKS = 40;
    public static double VEL_TOL_TPS = 60;

    // ---- Safety timeout ----
    public static double TIMEOUT_S = 4.0;

    // ======= Encoder / Robot constants =======
    // Set this to YOUR motor ticks per rev.
    public static double TICKS_PER_REV = 537.7;

    // Wheel + gear ratio
    public static double WHEEL_DIAMETER_IN = 3.78;  // change to yours
    public static double GEAR_RATIO = 1.0;          // output/input, usually 1.0 unless geared

    // Optional: scale correction after measuring (ex: 1.03 if it goes short)
    public static double DISTANCE_SCALE = 1.00;

    // Controllers
    private PIDController distancePID;
    private PIDController headingPID;

    private double ticksPerInch;

    @Override
    public void runOpMode() {
        // Dashboard telemetry + Driver Station telemetry together
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initHardware();

        telemetry.addLine("Ready. Press start.");
        telemetry.addLine("Open Dashboard at: http://192.168.43.1:8080/dash");
        telemetry.addData("IMU Detected", imuDetected);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Use current heading as the heading to hold (only if enabled + IMU detected)
        double holdHeadingDeg = (USE_HEADING_HOLD && imuDetected) ? getHeadingDeg() : 0.0;

        // One move for tuning (edit TEST_DISTANCE live in Dashboard)
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

        // Brake helps stop closer to target
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Compute ticks per inch
        ticksPerInch = DISTANCE_SCALE * (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

        resetAndRunEncoders();

        // IMU (optional)
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters p = new BNO055IMU.Parameters();
            p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu.initialize(p);
            imuDetected = true;
        } catch (Exception e) {
            imuDetected = false;
        }

        // Create controllers (we will update gains inside drive loop so dashboard edits apply)
        distancePID = new PIDController(DIST_P, DIST_I, DIST_D);
        headingPID  = new PIDController(HEAD_P, HEAD_I, HEAD_D);
        distancePID.setTolerance(POS_TOL_TICKS);
        headingPID.setTolerance(1.5);
    }

    /**
     * Drive forward/backward a given number of inches using PID on average encoder position.
     * @param inches positive forward, negative backward
     * @param holdHeadingDeg heading to maintain (only if enabled + IMU detected)
     */
    private void driveToDistancePID(double inches, double holdHeadingDeg) {
        resetAndRunEncoders();

        int targetTicks = (int) Math.round(inches * ticksPerInch);

        distancePID.reset();
        headingPID.reset();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < TIMEOUT_S) {

            // Update PID gains live (so Dashboard changes apply immediately)
            distancePID.setPID(DIST_P, DIST_I, DIST_D);
            headingPID.setPID(HEAD_P, HEAD_I, HEAD_D);
            distancePID.setTolerance(POS_TOL_TICKS);

            int pos = getAverageEncoderTicks();
            int error = targetTicks - pos;

            // Distance PID output: forward power
            double forward = distancePID.calculate(pos, targetTicks);

            // Clamp and add minimum power to overcome stiction
            forward = clamp(forward, -MAX_POWER, MAX_POWER);
            if (Math.abs(error) > POS_TOL_TICKS) {
                forward = applyMinPower(forward, MIN_POWER);
            }

            // Heading correction (turn) if enabled + IMU present
            double turn = 0.0;
            double heading = 0.0;
            double headingError = 0.0;

            boolean headingActive = USE_HEADING_HOLD && imuDetected;
            if (headingActive) {
                heading = getHeadingDeg();
                headingError = angleWrapDeg(holdHeadingDeg - heading);
                turn = headingPID.calculate(heading, holdHeadingDeg);
                turn = clamp(turn, -TURN_CLAMP, TURN_CLAMP);
            }

            // Apply to drivetrain (arcade, no strafe)
            setDrivePowers(forward, turn);

            // Stop conditions: within position tolerance AND slowed down
            double vel = getAverageVelocityTicksPerSec();
            boolean atPos = Math.abs(error) <= POS_TOL_TICKS;
            boolean settled = Math.abs(vel) <= VEL_TOL_TPS;

            // ====== TELEMETRY ======
            telemetry.addLine("===== DASHBOARD TUNING =====");
            telemetry.addLine("===== RAW ENCODERS =====");
            telemetry.addData("FL pos", frontLeftMotor.getCurrentPosition());
            telemetry.addData("FR pos", frontRightMotor.getCurrentPosition());
            telemetry.addData("BL pos", backLeftMotor.getCurrentPosition());
            telemetry.addData("BR pos", backRightMotor.getCurrentPosition());

            telemetry.addData("FL vel", "%.1f", frontLeftMotor.getVelocity());
            telemetry.addData("FR vel", "%.1f", frontRightMotor.getVelocity());
            telemetry.addData("BL vel", "%.1f", backLeftMotor.getVelocity());
            telemetry.addData("BR vel", "%.1f", backRightMotor.getVelocity());
            telemetry.addData("TestInches", inches);
            telemetry.addData("TicksPerInch", "%.3f", ticksPerInch);

            telemetry.addData("DIST P", DIST_P);
            telemetry.addData("DIST I", DIST_I);
            telemetry.addData("DIST D", DIST_D);

            telemetry.addData("USE_HEADING_HOLD", headingActive);
            telemetry.addData("HEAD P", HEAD_P);
            telemetry.addData("HEAD I", HEAD_I);
            telemetry.addData("HEAD D", HEAD_D);
            telemetry.addData("TURN_CLAMP", TURN_CLAMP);

            telemetry.addData("MAX_POWER", MAX_POWER);
            telemetry.addData("MIN_POWER", MIN_POWER);

            telemetry.addData("POS_TOL_TICKS", POS_TOL_TICKS);
            telemetry.addData("VEL_TOL_TPS", VEL_TOL_TPS);
            telemetry.addData("TIMEOUT_S", TIMEOUT_S);

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

            if (headingActive) {
                telemetry.addData("Heading", "%.1f", heading);
                telemetry.addData("HeadingErr", "%.1f", headingError);
            } else {
                telemetry.addData("IMU Detected", imuDetected);
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
        int fl = -frontLeftMotor.getCurrentPosition();  // negate because left motors are REVERSE
        int bl = backLeftMotor.getCurrentPosition();
        int fr =  frontRightMotor.getCurrentPosition();
        int br =  backRightMotor.getCurrentPosition();
        return (fl + fr + bl + br) / 4;
    }

    private double getAverageVelocityTicksPerSec() {
        double fl = -frontLeftMotor.getVelocity();      // negate for REVERSE
        double bl = backLeftMotor.getVelocity();
        double fr =  frontRightMotor.getVelocity();
        double br =  backRightMotor.getVelocity();
        return (fl + fr + bl + br) / 4.0;
    }

    /**
     * forward = + drives forward. turn = + turns right.
     * Arcade style on 4 motors (no strafe).
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
        return imu.getAngularOrientation().firstAngle;
    }

    private static double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
