package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by RobotK on 10/27/2016.
 */

@TeleOp (name = "TeleOp")
public class BasicTeleOp extends LinearOpMode {

    private Robot myRobot = null;

    public void mecanumDrive(double x1, double y1, double x2) {
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
    public void runOpMode() {
        myRobot = new Robot();
        myRobot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.right_bumper) {
                myRobot.collectorMotor.setPower(-1);
            } else {
                myRobot.collectorMotor.setPower(0);
            }

            if(gamepad1.b) {
                myRobot.launcherMotor.setPower(1);
            } else {
                myRobot.launcherMotor.setPower(0);
            }

            if(gamepad1.left_bumper) {
                myRobot.engageLauncher();
            } else if(gamepad1.left_trigger > 0) {
                myRobot.disengageLauncher();
            }


            if(gamepad1.x) {
                myRobot.lockLauncher(this);
            } else if (gamepad1.y) {
                myRobot.disengageLauncher();
            }
            telemetry.addData("Touch sensor", myRobot.launcherLimitTouchSensor.isPressed());
            telemetry.addData("Servo position", myRobot.launcherServo.getPosition());
            telemetry.update();

            myRobot.waitForTick(40);
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
