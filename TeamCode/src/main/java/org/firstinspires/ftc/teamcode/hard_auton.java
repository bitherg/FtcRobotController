package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "hard_auton_fixed")
public class hard_auton extends LinearOpMode {

    // Drive Motors
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // Other Motors
    DcMotorEx flyWheel, flyWheel_2, intake;

    // Servo
    Servo blocker;

    // Servo Positions
    static final double BLOCKER_DOWN = 0.0;
    static final double BLOCKER_UP   = 0.611;

    // Velocities
    static final double DRIVE_SPEED = 1000;
    static final double TURN_SPEED  = 800;
    static final double FLYWHEEL_VELOCITY = -1800;
    static final double INTAKE_VELOCITY   = -800;

    // Encoder constants (adjust if needed)
    static final double TICKS_PER_REV = 537.6; // goBILDA 312rpm example
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    @Override
    public void runOpMode() {

        // Hardware Mapping
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        flyWheel   = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel_2 = hardwareMap.get(DcMotorEx.class, "flyWheel_2");
        intake     = hardwareMap.get(DcMotorEx.class, "intake");

        blocker = hardwareMap.get(Servo.class, "blocker");

        // Reverse left side
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setupMotor(frontLeftMotor);
        setupMotor(backLeftMotor);
        setupMotor(frontRightMotor);
        setupMotor(backRightMotor);

        blocker.setPosition(BLOCKER_UP);

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        driveInches(-24);

        flyWheel.setVelocity(FLYWHEEL_VELOCITY);
        flyWheel_2.setVelocity(FLYWHEEL_VELOCITY);
        sleep(2000);

        blocker.setPosition(BLOCKER_DOWN);
        sleep(4000);

        intake.setVelocity(INTAKE_VELOCITY);
        driveInches(-8);

        intake.setVelocity(0);
        flyWheel.setVelocity(0);
        flyWheel_2.setVelocity(0);
    }


    private void driveInches(double inches) {

        int move = (int)(inches * TICKS_PER_INCH);

        setTargetPosition(move, move, move, move);
        runToPosition(DRIVE_SPEED);
    }

    private void turnDegrees(double degrees) {

        double robotDiameter = 15;
        double turnCircumference = Math.PI * robotDiameter;
        double inches = (degrees / 360.0) * turnCircumference;

        int move = (int)(inches * TICKS_PER_INCH);

        setTargetPosition(-move, -move, move, move);
        runToPosition(TURN_SPEED);
    }

    private void setTargetPosition(int fl, int bl, int fr, int br) {

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + fl);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + bl);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + fr);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + br);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runToPosition(double velocity) {

        frontLeftMotor.setVelocity(velocity);
        backLeftMotor.setVelocity(velocity);
        frontRightMotor.setVelocity(velocity);
        backRightMotor.setVelocity(velocity);

        while (opModeIsActive() &&
                (frontLeftMotor.isBusy() ||
                 backLeftMotor.isBusy() ||
                 frontRightMotor.isBusy() ||
                 backRightMotor.isBusy())) {
            idle();
        }

        stopDrive();

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopDrive() {
        frontLeftMotor.setVelocity(0);
        backLeftMotor.setVelocity(0);
        frontRightMotor.setVelocity(0);
        backRightMotor.setVelocity(0);
    }

    private void setupMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
