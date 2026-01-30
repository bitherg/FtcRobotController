package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "MyAutoOpMode")
public class autonomous extends LinearOpMode {
    Servo blocker;
    // Declare motors
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotorEx flyWheel;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        blocker = hardwareMap.get(Servo.class,"blocker");
        // Initialize the arm motor as DcMotorEx
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");

        // Set motor directions (reverse right side if needed)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);




    frontRightMotor.setPower(1);
    frontLeftMotor.setPower(1);
    sleep(420);
    frontRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
}
        // Example autonomous movement


    public void block(){
        blocker.setPosition(0);
    }
    public void unblock(){
        blocker.setPosition(.5);
    }
    public void spinwheel(){
        flyWheel.setPower(1);
    }
    public void stopwheel(){
        flyWheel.setPower(0);
    }
    public void enableForward(int x)
    {
        frontLeftMotor.setPower(0.05);
        backLeftMotor.setPower(0.05);
        frontRightMotor.setPower(0.05);
        backRightMotor.setPower(0.05);

        // Move forward for x ms
        sleep(x);

        // Stop all motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void enableLeft(int x)
    {
        frontLeftMotor.setPower(0.05);
        backLeftMotor.setPower(0.05);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0.);

        // Move forward for x ms
        sleep(x);

        // Stop all motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void enableRight(int x)
    {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0.05);
        backRightMotor.setPower(0.05);

        // Move forward for x ms
        sleep(x);

        // Stop all motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void enableLeftDiagonalForward(int x) {
        backLeftMotor.setPower(0.05);
        frontRightMotor.setPower(0.05);


        // Move forward for x seconds
        sleep(x);

        // Stop all motors
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }
    public void enableRightDiagonalForward(int x)
    {
        frontLeftMotor.setPower(0.05);
        backRightMotor.setPower(0.05);


        // Move forward for x seconds
        sleep(x);

        // Stop all motors
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void enableleftDiagonalbackwards(int x)
    {
        frontRightMotor.setPower(-0.05);
        backLeftMotor.setPower(-0.05);


        // Move forward for x seconds
        sleep( x);

        // Stop all motors
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
    public void enableRightDiagonalbackwards(int x)
    {
        frontLeftMotor.setPower(-0.05);
        backRightMotor.setPower(-0.05);


        // Move forward for x seconds
        sleep(x);

        // Stop all motors
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void enableBackwards(int x)
    {
        frontLeftMotor.setPower(-0.05);
        backLeftMotor.setPower(-0.05);
        frontRightMotor.setPower(-0.05);
        backRightMotor.setPower(-0.05);

        // Move forward for x seconds
        sleep(x);

        // Stop all motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
