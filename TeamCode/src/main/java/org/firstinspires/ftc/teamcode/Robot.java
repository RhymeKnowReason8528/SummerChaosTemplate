package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 * <p>
 * Note: the configuration of the servos is such that:
 * As the arm servo approaches 0, the arm position moves up (away from the floor).
 * As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class Robot {
    /* Public OpMode members. */
    DcMotor leftFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor rightRearMotor = null;

    DcMotor collectorMotor = null;

    DcMotor launcherMotor = null;
    Servo launcherServo;

    TouchSensor launcherLimitTouchSensor;
    ColorSensor colorSensor;
    ModernRoboticsI2cGyro gyro;

    /* Local OpMode members. */
    private HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    private LinearOpMode linearOpMode;
    private Thread pullbackThread;
    private DcMotor ENCODER_MOTOR; //initialize in init
    private boolean isLauncherPulledBack = false;
    final static double FAST_LIMIT_GYRO = 0.4;
    final static double SLOW_LIMIT_GYRO = 0.1;
    final double DRIVE_GAIN = .005;

    public boolean isLauncherPulledBack() {
        return isLauncherPulledBack;
    }

    public static enum Comparison{
        LESS_THAN{
            @Override
            public boolean evaluate(double x1, double x2) {
                return x1<x2;
            }
        },
        GREATER_THAN {
            @Override
            public boolean evaluate(double x1, double x2) {
                return x1>x2;
            }
        };

        public abstract boolean evaluate(double x1, double x2);
    }

    Comparison comparisonToUse;

    public void calibrateGyro () throws InterruptedException {
        gyro.calibrate();
        linearOpMode.telemetry.addData("calibrating", true);
        linearOpMode.telemetry.update();
        int calibrationTicks = 0;
        while(gyro.isCalibrating()){
            calibrationTicks++;
            waitForTick(200);
            linearOpMode.telemetry.addData("Gyro Calibration Ticks", calibrationTicks);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("calibrating", false);
        linearOpMode.telemetry.update();
    }

    /* Constructor */
    public Robot(LinearOpMode opmode) {
        linearOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) throws InterruptedException {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.dcMotor.get("left_front_drive");
        leftRearMotor = hwMap.dcMotor.get("left_rear_drive");
        rightFrontMotor = hwMap.dcMotor.get("right_front_drive");
        rightRearMotor = hwMap.dcMotor.get("right_rear_drive");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);

        collectorMotor = hwMap.dcMotor.get("collector");

        launcherMotor = hwMap.dcMotor.get("launcher_motor");
        launcherServo = hwMap.servo.get("launcher_servo");

        launcherLimitTouchSensor = hwMap.touchSensor.get("launcher_limit_sensor");
        colorSensor = hwMap.colorSensor.get("beacon_sensor");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro_sensor");

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
        collectorMotor.setPower(0);
        launcherMotor.setPower(0);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.

        calibrateGyro();

        ENCODER_MOTOR = leftFrontMotor;
    }


    public boolean turn (double targetHeading) throws InterruptedException {
        double currentHeading = gyro.getIntegratedZValue();
        double adjustedTargetHeading = targetHeading + currentHeading;

        double headingError;
        double driveSteering;

        if (currentHeading < adjustedTargetHeading) {
            comparisonToUse = Comparison.LESS_THAN;
        }
        else {
            comparisonToUse = Comparison.GREATER_THAN;
        }

        while (comparisonToUse.evaluate(currentHeading, adjustedTargetHeading) && linearOpMode.opModeIsActive()) {
            currentHeading = gyro.getIntegratedZValue();
            headingError = (adjustedTargetHeading - currentHeading);
            driveSteering = headingError * DRIVE_GAIN;

            rightFrontMotor.setPower(driveSteering);
            rightRearMotor.setPower(driveSteering);
            leftFrontMotor.setPower(-driveSteering);
            leftRearMotor.setPower(-driveSteering);

            //issue is with the following code, so I temporarily moved the power settings above it.
            if(driveSteering > FAST_LIMIT_GYRO) {
                driveSteering = FAST_LIMIT_GYRO;
                linearOpMode.telemetry.addData("Limiting", "At maximum speed positive");
            } else if (driveSteering < -FAST_LIMIT_GYRO) {
                driveSteering = -FAST_LIMIT_GYRO;
                linearOpMode.telemetry.addData("Limiting", "At maximum speed negative");
            } else if (driveSteering < SLOW_LIMIT_GYRO && driveSteering > -SLOW_LIMIT_GYRO) {
                linearOpMode.telemetry.addData("Limiting", "At minimum speed");
                if(comparisonToUse == Comparison.LESS_THAN) {
                    driveSteering = SLOW_LIMIT_GYRO;
                }
                else {
                    driveSteering = -SLOW_LIMIT_GYRO;
                }
            }

            linearOpMode.telemetry.addData("integratedZValue", gyro.getIntegratedZValue());
            linearOpMode.telemetry.addData("Current heading", currentHeading);
            linearOpMode.telemetry.addData("Target heading", adjustedTargetHeading);
            linearOpMode.telemetry.addData("Comparison being used", comparisonToUse);
            linearOpMode.telemetry.addData("Drive power", driveSteering);
            linearOpMode.telemetry.addData("Turn state", "In progress");
            linearOpMode.telemetry.update();
        }

        rightRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        leftFrontMotor.setPower(0);
        linearOpMode.telemetry.addData("Turn state", "done");
        linearOpMode.telemetry.update();

        return true;
    }
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void moveForward(double distance, double speed) {
        double encoderSubtractor = leftFrontMotor.getCurrentPosition();
        if (distance >= 0) {
            while (ENCODER_MOTOR.getCurrentPosition() - encoderSubtractor < distance && linearOpMode.opModeIsActive()) {
                leftFrontMotor.setPower(speed);
                leftRearMotor.setPower(speed);
                rightFrontMotor.setPower(speed);
                rightRearMotor.setPower(speed);
            }
            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);
        } else {
            while (ENCODER_MOTOR.getCurrentPosition() - encoderSubtractor > distance && linearOpMode.opModeIsActive()) {
                leftFrontMotor.setPower(-speed);
                leftRearMotor.setPower(-speed);
                rightFrontMotor.setPower(-speed);
                rightRearMotor.setPower(-speed);
            }
            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
        }
    }

    public void moveSideways(double distance, double speed) {
        double encoderSubtractor = leftFrontMotor.getCurrentPosition();
        if (distance >= 0) {
            while (ENCODER_MOTOR.getCurrentPosition() - encoderSubtractor < distance && linearOpMode.opModeIsActive()) {
                leftFrontMotor.setPower(speed);
                leftRearMotor.setPower(-speed);
                rightFrontMotor.setPower(-speed);
                rightRearMotor.setPower(speed);
            }
            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);
        } else {
            while (ENCODER_MOTOR.getCurrentPosition() - encoderSubtractor > distance && linearOpMode.opModeIsActive()) {
                leftFrontMotor.setPower(-speed);
                leftRearMotor.setPower(speed);
                rightFrontMotor.setPower(speed);
                rightRearMotor.setPower(-speed);
            }
            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);
        }
    }
    public void engageLauncher() {
        launcherServo.setPosition(0.1);
    }

    public void disengageLauncher() {
        launcherServo.setPosition(1);
        isLauncherPulledBack = false;
    }

    public void launchAndReload() {
        double initialRuntime = linearOpMode.getRuntime();
        if (pullbackThread == null || !pullbackThread.isAlive()) {
            pullbackThread = new Thread(new PullBackLauncherRunnable());
            pullbackThread.start();
        }
    }

    public boolean initLauncher(boolean initMethod) {
        engageLauncher();
        waitForTick(500);

        double beginingTime = linearOpMode.getRuntime();

        if (initMethod) {
            while (!launcherLimitTouchSensor.isPressed() && linearOpMode.getRuntime() < beginingTime + 1.67) {
                launcherMotor.setPower(1);
            }
        } else {
            while (!launcherLimitTouchSensor.isPressed() && linearOpMode.getRuntime() < beginingTime + 1.67 && linearOpMode.opModeIsActive()) {
                launcherMotor.setPower(1);
            }
        }
        launcherMotor.setPower(0);
        isLauncherPulledBack = true;
//        opmode.telemetry.addData("status", "Launcher ready to fire");
//        opmode.telemetry.update();

        Log.i("RKR", "Pullback took " + (linearOpMode.getRuntime() - beginingTime));

        return launcherLimitTouchSensor.isPressed();
    }


    private class PullBackLauncherRunnable implements Runnable {

        @Override
        public void run() {
            disengageLauncher();
            waitForTick(500);
            initLauncher(false);
        }
    }
}
