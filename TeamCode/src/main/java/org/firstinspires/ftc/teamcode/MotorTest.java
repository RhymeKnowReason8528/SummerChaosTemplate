package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by RobotK on 10/27/2016.
 */

@TeleOp (name = "Motor Encoder Test")
public class MotorTest extends LinearOpMode {

    Robot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        myRobot = new Robot(this);
        myRobot.init(hardwareMap);

        myRobot.initLauncher(true);

        myRobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {//Right front
                myRobot.rightFrontMotor.setPower(1);
                telemetry.addData("Right front encoder value", myRobot.rightFrontMotor.getCurrentPosition());
            } else {
                myRobot.rightFrontMotor.setPower(0);
            }

            if(gamepad1.b) {//Right rear
                myRobot.rightRearMotor.setPower(1);
                telemetry.addData("Right rear encoder value", myRobot.rightRearMotor.getCurrentPosition());
            } else {
                myRobot.rightRearMotor.setPower(0);
            }

            if(gamepad1.a) {//Left front
                myRobot.leftFrontMotor.setPower(1);
                telemetry.addData("Left front encoder value", myRobot.leftFrontMotor.getCurrentPosition());
            } else {
                myRobot.leftFrontMotor.setPower(0);
            }

            if(gamepad1.x) {//Left rear
                myRobot.leftRearMotor.setPower(1);
                telemetry.addData("Left rear encoder value", myRobot.leftRearMotor.getCurrentPosition());
            } else {
                myRobot.leftRearMotor.setPower(0);
            }


            if(gamepad1.left_bumper) {
                myRobot.leftFrontMotor.setPower(1);
            }

            telemetry.update();

            myRobot.waitForTick(40);
        }
    }
}
