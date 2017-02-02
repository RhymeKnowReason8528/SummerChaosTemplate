package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by RobotK on 10/27/2016.
 */

@TeleOp (name = "ValueTweak")
public class ValueTweak extends LinearOpMode {


    private Robot myRobot;



    @Override
    public void runOpMode() throws InterruptedException {
        myRobot = new Robot(this);
        myRobot.init(hardwareMap);

        myRobot.initLauncher(true);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.right_bumper) {
                myRobot.launcherServo.setPosition(myRobot.launcherServo.getPosition() + 0.01);
                while(gamepad1.right_bumper && opModeIsActive()) {
                }
            } else if(gamepad1.right_trigger > 0.5) {
                myRobot.launcherServo.setPosition(myRobot.launcherServo.getPosition() - 0.01);
                while(gamepad1.right_trigger > 0.5) {
                }
            }

            telemetry.addData("Servo position", myRobot.launcherServo.getPosition());

            telemetry.update();
            myRobot.waitForTick(20);
       }
    }
}
