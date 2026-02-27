package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "hardautonv2far", group = "auto")
public class hardautonv2far extends LinearOpMode {

    DcMotorEx leftFront, rightFront, leftRear, rightRear, flyWheel, flyWheel_2, intake;
    Servo blocker;

    private final ElapsedTime timer = new ElapsedTime();

    // Tune these for your robot
    private static final double TUNED_VOLTAGE = 13.0; // voltage you tuned powers at
    private static final double DRIVE_SCALE_MAX = 1.15; // cap so it doesn't overboost too much

    // Feeder positions (0..1)
    private static final double FEED_RETRACT = 0.0;
    private static final double FEED_PUSH = 110.0 / 180.0;

    @Override
    public void runOpMode() {

        // ----- Hardware Map -----
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel_2 = hardwareMap.get(DcMotorEx.class, "flyWheel_2");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        blocker = hardwareMap.get(Servo.class, "blocker");
        //blocker.setPosition(FEED_RETRACT);

        // ----- Motor Setup -----
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheel_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Directions (adjust as needed)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // If flywheels face opposite directions, you may need:
        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        flyWheel_2.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Ready. Press START!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        timer.reset();
        blocker.setPosition(FEED_PUSH);
        setDrivePowers(-1,-1,-1,-1);
        sleep(1250);
        setDrivePowers(0,0.5,0,0.5);
        sleep(500);
        stopDrive();
        flyWheel.setPower(0.8);
        flyWheel_2.setPower(0.8);
        sleep(2000);
        intake.setPower(-1);
        sleep(2000);
        flyWheel.setPower(0.7);
        flyWheel_2.setPower(0.7);
        sleep(2000);
        blocker.setPosition(FEED_RETRACT);
        sleep(1000);

        // Stop everything after shot
        intake.setPower(0);
        flyWheel.setPower(0);
        flyWheel_2.setPower(0);

        blocker.setPosition(FEED_PUSH);

        blocker.setPosition(FEED_RETRACT);
        boolean shotDone = false;

        // Example: drive forward first
//        while (timer.seconds() < 1.200) {
//            drive(0.5, 0.0, 0.0);
//        }
        // Shoot once during this window
//        stopDrive();
  //      shootOnceVoltageComp(1);

        //drive(0.0, 0.0, 0.5);


        //shootOnceVoltageComp(1.0, 0.50); // tunedPower, spinUpSeconds

        stopDrive();
    }

// ---------------- Voltage Helpers ----------------

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) result = Math.min(result, v);
        }
        return (result == Double.POSITIVE_INFINITY) ? 0.0 : result;
    }

    /**
     * Scale a tuned power value (tuned at TUNED_VOLTAGE) to current voltage.
     */
    private double compensatePower(double tunedPower) {
        double v = getBatteryVoltage();
        if (v <= 1.0) return tunedPower; // fallback if sensor fails
        double scaled = tunedPower * (TUNED_VOLTAGE / v);
        // Clamp to [0,1]
        return Math.max(0.0, Math.min(1.0, scaled));
    }

    /**
     * Optional: scale drive a little, but not too aggressively
     */
    private double compensateDrive(double tunedPower) {
        double v = getBatteryVoltage();
        if (v <= 1.0) return tunedPower;
        double scaled = tunedPower * (TUNED_VOLTAGE / v);
        // Drive usually shouldn't be boosted as hard as flywheels
        scaled = Math.max(-DRIVE_SCALE_MAX, Math.min(DRIVE_SCALE_MAX, scaled));
        return Math.max(-1.0, Math.min(1.0, scaled));
    }

// ---------------- Drive ----------------

    private void drive(double forward, double strafe, double turn) {
        // Voltage-compensate the inputs (simple + effective for time-based auton)
        forward = compensateDrive(forward);
        strafe = compensateDrive(strafe);
        turn = compensateDrive(turn);

        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lr = forward - strafe + turn;
        double rr = forward + strafe - turn;

        setDrivePowers(lf, rf, lr, rr);
    }

    private void setDrivePowers(double lf, double rf, double lr, double rr) {
        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr)))));

        lf /= max;
        rf /= max;
        lr /= max;
        rr /= max;

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
    }


    private void stopDrive() {
        setDrivePowers(0, 0, 0, 0);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftRear.setZeroPowerBehavior(behavior);
        rightRear.setZeroPowerBehavior(behavior);
    }

// ---------------- Flywheel / Shooting ----------------

    private void setFlywheelPower(double tunedPower) {
        // Apply voltage compensation to maintain similar RPM across batteries
        double p = compensatePower(tunedPower);
        flyWheel.setPower(p);
        flyWheel_2.setPower(p);
    }

    /**
     * Shoots exactly one ring (blocking)
     */
    private void shootOnceVoltageComp(double tunedFlywheelPower) {


        setFlywheelPower(tunedFlywheelPower);
        sleep(2000);
        intake.setPower(-1);
        sleep(2000);
        blocker.setPosition(FEED_RETRACT);
        sleep(1000);

        // Stop everything after shot
        intake.setPower(0);
        flyWheel.setPower(0);
        flyWheel_2.setPower(0);

        blocker.setPosition(FEED_PUSH);

    }
}