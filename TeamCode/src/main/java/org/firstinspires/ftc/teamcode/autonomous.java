package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "MyAutoOpMode")
public class autonomous extends LinearOpMode {

    // Declare motors
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotorEx armMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Initialize the arm motor as DcMotorEx
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");

        // Set motor directions (reverse right side if needed)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set arm motor braking and mode
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the start button
        waitForStart();

        // Example autonomous movement
        frontLeftMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backRightMotor.setPower(0.5);

        // Move forward for 1 second
        sleep(1000);

        // Stop all motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        // Run the arm motor
        armMotor.setPower(0.5);
        sleep(1000);
        armMotor.setPower(0);
    }
}
