package org.firstinspires.ftc.teamcode;

import android.speech.tts.TextToSpeech;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

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
    RkrTTS tts;

    /* Local OpMode members. */
    private HardwareMap hwMap = null;
    private LinearOpMode linearOpMode;
    private Thread pullbackThread;
    private DcMotor ENCODER_MOTOR; //initialize in init
    private boolean isLauncherPulledBack = false;
    final static double FAST_LIMIT_GYRO = 0.8;
    final static double SLOW_LIMIT_GYRO = 0.2;
    final double DRIVE_GAIN = .005;
    final double LAUNCHER_ENGAGED = 0.1;
    final double LAUNCHER_DISENGAGED = 1;

    enum CollectorState {
        RUNNING_FORWARD,
        RUNNING_BACKWARD,
        STOPPED
    }

    CollectorState collectorState = CollectorState.STOPPED;

    public boolean isLauncherPulledBack() {
        return isLauncherPulledBack;
    }

    public static enum Comparison {
        LESS_THAN {
            @Override
            public boolean evaluate(double x1, double x2) {
                return x1 < x2;
            }
        },
        GREATER_THAN {
            @Override
            public boolean evaluate(double x1, double x2) {
                return x1 > x2;
            }
        };

        public abstract boolean evaluate(double x1, double x2);
    }

    Comparison comparisonToUse;

    public void calibrateGyro() throws InterruptedException {
        gyro.calibrate();
        ElapsedTime elapsedTime = new ElapsedTime();
        linearOpMode.telemetry.addData("calibrating", true);
        linearOpMode.telemetry.update();
        int calibrationTicks = 0;
        while (gyro.isCalibrating()) {
            calibrationTicks++;
            waitForTick(200, elapsedTime);
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
        hwMap = ahwMap;

        tts = new RkrTTS();
        tts.init(hwMap.appContext);

        // Define and Initialize Motors
        leftFrontMotor = hwMap.dcMotor.get("left_front_drive");
        leftRearMotor = hwMap.dcMotor.get("left_rear_drive");
        rightFrontMotor = hwMap.dcMotor.get("right_front_drive");
        rightRearMotor = hwMap.dcMotor.get("right_rear_drive");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);

        collectorMotor = hwMap.dcMotor.get("collector");
        collectorMotor.setDirection(DcMotor.Direction.REVERSE); // Remove this line unless using the NeveRest 3.7 motor.

        launcherMotor = hwMap.dcMotor.get("launcher_motor");
        launcherServo = hwMap.servo.get("launcher_servo");

        launcherLimitTouchSensor = hwMap.touchSensor.get("launcher_limit_sensor");
        colorSensor = hwMap.colorSensor.get("beacon_sensor");
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro_sensor");

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
        collectorMotor.setPower(0);
        launcherMotor.setPower(0);


        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

  /*      rightFrontMotor.setMaxSpeed(2770);
        rightRearMotor.setMaxSpeed(2770);
        leftFrontMotor.setMaxSpeed(2770);
        leftRearMotor.setMaxSpeed(2770);*/

        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.

        tts.speakWords("Battery level is " + String.format(Locale.US, "%.2f", getBatteryVoltage()) + " Volts");
        tts.speakWords("Go RhymeKnowReason!");

        calibrateGyro();


        ENCODER_MOTOR = leftFrontMotor;
    }


    public boolean turn(double targetHeading) throws InterruptedException {
        double currentHeading = gyro.getIntegratedZValue();
        double adjustedTargetHeading = targetHeading + currentHeading;

        double headingError;
        double driveSteering;

        if (currentHeading < adjustedTargetHeading) {
            comparisonToUse = Comparison.LESS_THAN;
        } else {
            comparisonToUse = Comparison.GREATER_THAN;
        }

        gyro.getIntegratedZValue();

        while (comparisonToUse.evaluate(currentHeading, adjustedTargetHeading) && linearOpMode.opModeIsActive()) {
            currentHeading = gyro.getIntegratedZValue();
            headingError = (adjustedTargetHeading - currentHeading);
            driveSteering = headingError * DRIVE_GAIN;

//            linearOpMode.telemetry.addData("integratedZValue 1", gyro.getIntegratedZValue());
//            linearOpMode.telemetry.addData("Current heading 1", currentHeading);
//            linearOpMode.telemetry.addData("Target heading 1", adjustedTargetHeading);
//            linearOpMode.telemetry.addData("Comparison being used 1", comparisonToUse);
//            linearOpMode.telemetry.addData("Turn state 1", "In progress");


            //issue is with the following code, so I temporarily moved the power settings above it.
            if (driveSteering > FAST_LIMIT_GYRO) {
                driveSteering = FAST_LIMIT_GYRO;
                linearOpMode.telemetry.addData("Limiting", "At maximum speed positive");
            } else if (driveSteering < -FAST_LIMIT_GYRO) {
                driveSteering = -FAST_LIMIT_GYRO;
                linearOpMode.telemetry.addData("Limiting", "At maximum speed negative");
            } else if (driveSteering < SLOW_LIMIT_GYRO && driveSteering > -SLOW_LIMIT_GYRO) {
                linearOpMode.telemetry.addData("Limiting", "At minimum speed");
                if (comparisonToUse == Comparison.LESS_THAN) {
                    driveSteering = SLOW_LIMIT_GYRO;
                } else {
                    driveSteering = -SLOW_LIMIT_GYRO;
                }
            }

            rightFrontMotor.setPower(driveSteering);
            rightRearMotor.setPower(driveSteering);
            leftFrontMotor.setPower(-driveSteering);
            leftRearMotor.setPower(-driveSteering);

//            linearOpMode.telemetry.addData("integratedZValue 2", gyro.getIntegratedZValue());
//            linearOpMode.telemetry.addData("Current heading 2", currentHeading);
//            linearOpMode.telemetry.addData("Target heading 2", adjustedTargetHeading);
//            linearOpMode.telemetry.addData("Comparison being used 2", comparisonToUse);
//            linearOpMode.telemetry.addData("Drive power 2", driveSteering);
//            linearOpMode.telemetry.addData("Turn state 2", "In progress");
        }

        rightRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        leftFrontMotor.setPower(0);

        return true;
    }

    /**
     * waitForTick should only be used within a loop (use Thread.sleep for other purposes). This is so that in a loop it waits
     * at least for the time put in, but if the loop has already taken that long it will just pass over the method
     *
     * @param periodMs The length of time you want each loop cycle to take
     */
    public void waitForTick(long periodMs, ElapsedTime elapsedTime) {

        long remaining = periodMs - (long) elapsedTime.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        //Reset the cycle clock for the next pass.
        elapsedTime.reset();
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
            rightRearMotor.setPower(0);
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
        launcherServo.setPosition(LAUNCHER_ENGAGED);
    }

    public void disengageLauncher() throws InterruptedException {
        launcherServo.setPosition(LAUNCHER_DISENGAGED);
        Thread.sleep(500);
        isLauncherPulledBack = false;
    }

    public void launchAndReload() {
        if (pullbackThread == null || !pullbackThread.isAlive()) {
            pullbackThread = new Thread(new PullBackLauncherRunnable());
            pullbackThread.start();
        }
    }

    public boolean initLauncher(boolean initMethod) throws InterruptedException {
        engageLauncher();
        Thread.sleep(500);

        ElapsedTime elapsedTime = new ElapsedTime();

        double beginingTime = linearOpMode.getRuntime();

        if (initMethod) {
            while (!launcherLimitTouchSensor.isPressed()) {//It took 1.67 seconds to pull back the launcher
                launcherMotor.setPower(1);
                waitForTick(10, elapsedTime);
            }
        } else {
            while (!launcherLimitTouchSensor.isPressed() && linearOpMode.opModeIsActive()) {
                launcherMotor.setPower(1);
                waitForTick(10, elapsedTime);
            }
        }
        launcherMotor.setPower(0);
        isLauncherPulledBack = true;
        linearOpMode.telemetry.addData("status", "Launcher ready to fire");
        linearOpMode.telemetry.update();

        Log.i("RKR", "Pullback took " + (linearOpMode.getRuntime() - beginingTime));

        return launcherLimitTouchSensor.isPressed();
    }

    public void runCollector() {
        if (collectorState == CollectorState.RUNNING_FORWARD) {//Used to be at 1 and -1
            collectorMotor.setPower(0.75);
        } else if (collectorState == CollectorState.RUNNING_BACKWARD) {
            collectorMotor.setPower(-0.75);
        } else if (collectorState == CollectorState.STOPPED) {
            collectorMotor.setPower(0);
        }
    }

    private class PullBackLauncherRunnable implements Runnable {

        @Override
        public void run() {
            try {
                disengageLauncher();
                initLauncher(false);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            sensor.getDeviceName();
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
