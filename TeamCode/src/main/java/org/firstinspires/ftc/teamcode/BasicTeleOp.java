package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by RobotK on 10/27/2016.
 */

@TeleOp (name = "TeleOp")
public class BasicTeleOp extends LinearOpMode {


    private Robot myRobot;

    public void mecanumDrive(double x1, double y1, double x2) {//testing GitHub
        x1 = scaleInput(x1);
        y1 = scaleInput(y1);

        double FL = y1 - x1 - x2;
        double FR = y1 + x1 + x2;
        double BL = y1 + x1 - x2;
        double BR = y1 - x1 + x2;

        double mv = max(Math.abs(FL), Math.abs(FR), Math.abs(BL), Math.abs(BR));
        if (Math.abs(mv) > 1) {
            FL = FL / mv;
            FR = FR / mv;
            BL = BL / mv;
            BR = BR / mv;
        }
            myRobot.leftFrontMotor.setPower(FL);
            myRobot.leftRearMotor.setPower(BL);
            myRobot.rightFrontMotor.setPower(FR);
            myRobot.rightRearMotor.setPower(BR);
    }

    private double max(double... args) {
        double m = 0;

        for (int i = 0; i < args.length; i++) {
            if (args[i] > m) {
                m = args[i];
            }
        }

        return m;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        myRobot = new Robot(this);
        myRobot.init(hardwareMap);

        myRobot.initLauncher(true);

        double elapsedTimeAtLoopStart;

        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            myRobot.runCollector();

            if(gamepad2.right_bumper || gamepad1.right_bumper) {
                myRobot.collectorState = Robot.CollectorState.RUNNING_FORWARD;
            } else if(gamepad2.right_trigger > 0.5 || gamepad1.right_trigger > 0.5) {
                myRobot.collectorState = Robot.CollectorState.STOPPED;
            } else if (gamepad2.left_bumper || gamepad1.left_bumper) {
                myRobot.collectorState = Robot.CollectorState.RUNNING_BACKWARD;
            } else if (myRobot.collectorState == Robot.CollectorState.RUNNING_BACKWARD) {
                myRobot.collectorState = Robot.CollectorState.STOPPED;
            }

            if (gamepad1.y) {
                myRobot.launchAndReload();
            }

          /*  if(!myRobot.isLauncherPulledBack()) {

            }*/


            telemetry.addData("Touch sensor", myRobot.launcherLimitTouchSensor.isPressed());
            telemetry.addData("Collector state", myRobot.collectorState);

            if(myRobot.isLauncherPulledBack()) {
                telemetry.addData("Launcher state", "Ready");
            } else {
                telemetry.addData("Launcher state", "Launching");
            }

            telemetry.update();
            myRobot.waitForTick(20);
       }
    }

    private double scaleInput(double dVal) {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60,
                                0.72, 0.85, 1.00, 1.00};

        int index = (int) (dVal * 15.0);
        if(index < 0) {
            index = -index;
        } else if (index > 15) {
            index = 15;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
