package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "servo_up_flywheel_on", group = "auto")
public class hard_auton extends LinearOpMode {

    // Shooter motors
    private DcMotorEx flyWheel;
    private DcMotorEx flyWheel_2;

    // Feeder/ramp servo
    private Servo blocker;

    // Tune these
    private static final double FEED_RETRACT = 0.0;   // down
    private static final double FEED_PUSH    = 0.611; // up

    // Flywheel velocity (ticks/sec) - tune
    private static final double FLYWHEEL_VEL_1 = -1800;
    private static final double FLYWHEEL_VEL_2 =  1800;

    @Override
    public void runOpMode() {

        flyWheel   = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel_2 = hardwareMap.get(DcMotorEx.class, "flyWheel_2");
        blocker    = hardwareMap.get(Servo.class, "blocker");

        // Encoder mode for velocity control
        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flyWheel_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Start servo down
        blocker.setPosition(FEED_RETRACT);

        telemetry.addLine("Ready: will raise servo then spin flywheels");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 1) Raise servo (bring balls up the ramp)
        blocker.setPosition(FEED_PUSH);
        sleep(300); // optional: give servo time to move

        // 2) Start flywheels and keep them running
        flyWheel.setVelocity(FLYWHEEL_VEL_1);
        flyWheel_2.setVelocity(FLYWHEEL_VEL_2);

        // Keep running until auton ends / stop pressed
        while (opModeIsActive()) {
            telemetry.addData("Flywheel 1 vel", "%.0f", flyWheel.getVelocity());
            telemetry.addData("Flywheel 2 vel", "%.0f", flyWheel_2.getVelocity());
            telemetry.update();
        }

        // Stop (if match ends or stop pressed)
        flyWheel.setVelocity(0);
        flyWheel_2.setVelocity(0);
    }
}