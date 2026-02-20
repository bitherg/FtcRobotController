
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "hard_auton")
public class hard_auton extends LinearOpMode {

    // ---------------- Drive Motors ----------------
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;

    // ---------------- Other Motors ----------------
    DcMotorEx flyWheel;
    DcMotorEx flyWheel_2;
    DcMotorEx intake;

    // ---------------- Servo ----------------
    Servo blocker;

    // Servo positions
    static final double BLOCKER_DOWN = 0.0;
    static final double BLOCKER_UP   = 0.611; // ≈ 0.611, make 360 degree turn Asap

    // ---------------- Velocity Constants ----------------
    static final double DRIVE_VELOCITY     = 1200;  // ticks/sec
    static final double SLOW_VELOCITY      = 300;
//   static final double FLYWHEEL_VELOCITY  = -1800;
//  static final double INTAKE_VELOCITY    = -800;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- Hardware Mapping ----------------
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        flyWheel   = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel_2 = hardwareMap.get(DcMotorEx.class, "flyWheel_2");
        intake     = hardwareMap.get(DcMotorEx.class, "intake");

        blocker = hardwareMap.get(Servo.class, "blocker");

        // ---------------- Motor Directions ----------------
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- Encoder Setup ----------------
        setupMotor(frontLeftMotor);
        setupMotor(backLeftMotor);
        setupMotor(frontRightMotor);
        setupMotor(backRightMotor);

        setupMotor(flyWheel);
        setupMotor(flyWheel_2);
        setupMotor(intake);

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        // ---------------- WAIT FOR START ----------------
        waitForStart();

//        if (isStopRequested()) return;

        // ---------------- Autonomous Routine ----------------
//        blocker.setPosition(BLOCKER_UP);
//        sleep(200);
//
//        enableForward(600);
//        sleep(300);
//
//        flyWheel.setVelocity(-1);
//        flyWheel_2.setVelocity(1);
//        sleep(1000);
//
//        intake.setVelocity(1);
//        blocker.setPosition(BLOCKER_DOWN);
//        sleep(1000);

//auton set position of the bot towards the corner of the triangle, parallel to it
        blocker.setPosition(BLOCKER_DOWN);
        sleep(1000);
        enableBackwards(600);
        sleep(1000);
        flyWheel.setVelocity(-1);
        flyWheel_2.setVelocity(1);
        sleep(2000);
        intake.setVelocity(1);
        sleep(1000);
        blocker.setPosition(BLOCKER_UP);
//      blocker.setPosition(BLOCKER_UP);
//      enableForward(2000); figure out why there is major drift only during auton.
//      enableBackward(2000);
//      turnLeft(500);
//      flyWheel.setVelocity(FLYWHEEL_VELOCITY);
//      flyWheel_2.setVelocity(FLYWHEEL_VELOCITY);
//      2 BALL
        //yash's code
        //make sure we are all productive at the mill and coding
        //also whenever code needs assistance Vivaan can help
//        .setVelocity(0);
//        flyWheel.setVelocity(0);
//        flyWheel_2.setVelocity(0);
////going based of estimation(change when test)
//        enableForward(200);
//        turnLeft(100);
//        intake.setVelocity(INTAKE_VELOCITY);
//        enableBackwards(200);
//        intake.setVelocity(0);
//        enableForward(200);
//        turnRight(100);
//        enableBackwards(200);
//        flyWheel.setVelocity(FLYWHEEL_VELOCITY);
//        flyWheel_2.setVelocity(FLYWHEEL_VELOCITY);
//        intake.setVelocity(INTAKE_VELOCITY);
//        intake.setVelocity(0);
//        blocker.setPosition(BLOCKER_DOWN);









        // ---------------- Stop Everything ----------------
        stopDrive();
        intake.setVelocity(0);
        flyWheel.setVelocity(0);
        flyWheel_2.setVelocity(0);
    }

    // ================= Movement Methods =================
    public void turnLeft(int timeMs)
    {
        frontLeftMotor.setPower(1);
        backRightMotor.setPower(1);
        sleepSafe(timeMs);
        stopDrive();

    }
    public void turnRight(int timeMs)
    {
        frontLeftMotor.setPower(1);
        backRightMotor.setPower(1);
        sleepSafe(timeMs);
        stopDrive();

    }
    public void enableForward(int timeMs) {
        setAllDriveVelocity(DRIVE_VELOCITY);
        sleepSafe(timeMs);
        stopDrive();
    }

    public void enableBackwards(int timeMs) {
        setAllDriveVelocity(-DRIVE_VELOCITY);
        sleepSafe(timeMs);
        stopDrive();
    }
    //strafe left
    public void enableLeft(int timeMs) {
        frontLeftMotor.setVelocity(SLOW_VELOCITY);
        backLeftMotor.setVelocity(SLOW_VELOCITY);
        frontRightMotor.setVelocity(0);
        backRightMotor.setVelocity(0);
        sleepSafe(timeMs);
        stopDrive();
    }
    //strafe right
    public void enableRight(int timeMs) {
        frontLeftMotor.setVelocity(0);
        backLeftMotor.setVelocity(0);
        frontRightMotor.setVelocity(SLOW_VELOCITY);
        backRightMotor.setVelocity(SLOW_VELOCITY);
        sleepSafe(timeMs);
        stopDrive();
    }

    public void enableLeftDiagonalForward(int timeMs) {
        backLeftMotor.setVelocity(SLOW_VELOCITY);
        frontRightMotor.setVelocity(SLOW_VELOCITY);
        sleepSafe(timeMs);
        stopDrive();
    }

    public void enableRightDiagonalForward(int timeMs) {
        frontLeftMotor.setVelocity(SLOW_VELOCITY);
        backRightMotor.setVelocity(SLOW_VELOCITY);
        sleepSafe(timeMs);
        stopDrive();
    }

    public void enableLeftDiagonalBackwards(int timeMs) {
        frontRightMotor.setVelocity(-SLOW_VELOCITY);
        backLeftMotor.setVelocity(-SLOW_VELOCITY);
        sleepSafe(timeMs);
        stopDrive();
    }

    public void enableRightDiagonalBackwards(int timeMs) {
        frontLeftMotor.setVelocity(-SLOW_VELOCITY);
        backRightMotor.setVelocity(-SLOW_VELOCITY);
        sleepSafe(timeMs);
        stopDrive();
    }

    // ================= Helper Methods =================

    private void setAllDriveVelocity(double velocity) {
        frontLeftMotor.setVelocity(velocity);
        backLeftMotor.setVelocity(velocity);
        frontRightMotor.setVelocity(velocity);
        backRightMotor.setVelocity(velocity);
    }

    private void stopDrive() {
        setAllDriveVelocity(0);
    }

    private void setupMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void sleepSafe(int ms) {
        if (opModeIsActive()) sleep(ms);
    }
}
