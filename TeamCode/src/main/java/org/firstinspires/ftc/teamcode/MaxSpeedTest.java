/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
//@Autonomous(name = "Sensor: MR Color", group = "Sensor")
@Autonomous(name = "Max speed test")
public class MaxSpeedTest extends LinearOpMode {

  private Robot myRobot = new Robot(this);

 // ColorSensor colorSensor;    // Hardware Device Object



  @Override
  public void runOpMode() throws InterruptedException {
    myRobot.init(hardwareMap);

    myRobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    myRobot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    myRobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    myRobot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    waitForStart();


    while (opModeIsActive()) {

      double time = this.getRuntime();

      while(this.getRuntime() - time < 5 && opModeIsActive()) {
        myRobot.rightFrontMotor.setPower(1);
        myRobot.rightRearMotor.setPower(1);
        myRobot.leftFrontMotor.setPower(1);
        myRobot.leftRearMotor.setPower(1);
      }

      myRobot.rightFrontMotor.setPower(0);
      myRobot.rightRearMotor.setPower(0);
      myRobot.leftFrontMotor.setPower(0);
      myRobot.leftRearMotor.setPower(0);

      while(opModeIsActive()) {
        telemetry.addData("Right front max speed", myRobot.rightFrontMotor.getCurrentPosition() / 5);
        telemetry.addData("Right rear max speed", myRobot.rightRearMotor.getCurrentPosition() / 5);
        telemetry.addData("Left front max speed", myRobot.leftFrontMotor.getCurrentPosition() / 5);
        telemetry.addData("Left rear max speed", myRobot.leftRearMotor.getCurrentPosition() / 5);

        telemetry.update();
      }

      requestOpModeStop();
    }
  }
}
