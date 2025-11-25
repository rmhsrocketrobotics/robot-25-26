package org.firstinspires.ftc.teamcode.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;

@TeleOp(group = "!main")

public class MainTeleop extends LinearOpMode{
    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap); // wheels
        Spindex spindex = new Spindex(hardwareMap, telemetry); // drumServo, intake, flick
        Outtake outtake = new Outtake(hardwareMap, true); // outtake, hoodServo (doesn't exist yet TODO make this exist)
        Vision vision = new Vision(hardwareMap); // camera

        Gamepad gamepad1Last = new Gamepad();
        Gamepad gamepad2Last = new Gamepad();

        //random ahh imu stuff dw
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        telemetry.setMsTransmissionInterval(200); //default 250
        //telemetry.setNumDecimalPlaces(0, 5);

        waitForStart();

        spindex.init();
        //vision.detectObeliskBlocking(drivetrain, telemetry);

        while (opModeIsActive()) {
            // GAMEPAD 1 CODE:
            drivetrain.setDrivetrainPower(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            // GAMEPAD 1 CODE END

            // GAMEPAD 2 CODE:
            double desiredTPS = -gamepad2.left_stick_y * 10000;
            outtake.setOuttakeVelocityTPS(desiredTPS);
            telemetry.addData("desiredTPS", desiredTPS);

            spindex.intake.setPower(gamepad2.right_trigger);
            spindex.flick.setPower(gamepad2.left_trigger);

            if (gamepad2.y && !gamepad2Last.y) {
                spindex.incrementDrumPosition();
            }
            if (gamepad2.x && !gamepad2Last.x) {
                spindex.setDrumState("intake", 0);
            }
            if (gamepad2.left_bumper && !gamepad2Last.left_bumper) {
                spindex.flickNextBall("purple");
            }
            if (gamepad2.right_bumper && !gamepad2Last.right_bumper) {
                spindex.flickNextBall("green");
            }

            gamepad2Last.copy(gamepad2);

            spindex.update(true);
            // GAMEPAD 2 CODE END

            //drivetrain.printTelemetry(telemetry);
            //outtake.printTelemetry(telemetry);
            //telemetry.addData("facing direction radians", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            //telemetry.addData("angular velocity radians", imu.getRobotAngularVelocity(AngleUnit.RADIANS));
            //telemetry.addData("facing direction degrees", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            //telemetry.addData("angular velocity degrees", imu.getRobotAngularVelocity(AngleUnit.DEGREES));

            telemetry.update();
        }
    }
}

class Drivetrain {
    public DcMotor flMotor;
    public DcMotor frMotor;
    public DcMotor blMotor;
    public DcMotor brMotor;
    Drivetrain(HardwareMap hardwareMap) {
        flMotor = hardwareMap.get(DcMotor.class, "fl");
        frMotor = hardwareMap.get(DcMotor.class, "fr");
        blMotor = hardwareMap.get(DcMotor.class, "bl");
        brMotor = hardwareMap.get(DcMotor.class, "br");

        //change the direction of drivetrain motors so they're all facing the same way
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        //make the motors brake
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset encoder values
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //run w/o velocity pid
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivetrainPower(double yPower, double xPower, double rPower) {
        //equations to move the robot
        flMotor.setPower(yPower + xPower + rPower);
        frMotor.setPower(yPower - xPower - rPower);
        blMotor.setPower(yPower - xPower + rPower);
        brMotor.setPower(yPower + xPower - rPower);
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("fldrivetrain is ", flMotor.getCurrentPosition());
        telemetry.addData("frdrivetrain is ", frMotor.getCurrentPosition());
        telemetry.addData("bldrivetrain is ", blMotor.getCurrentPosition());
        telemetry.addData("brdrivetrain is ", brMotor.getCurrentPosition());
    }
}

class Outtake {
    DcMotorEx outtake1;
    DcMotorEx outtake2;
    PIDFCoefficients MOTOR_VELO_PID;
    VoltageSensor batteryVoltageSensor;
    Outtake(HardwareMap hardwareMap, boolean useVeloPID) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        if (useVeloPID) {
            MOTOR_VELO_PID = new PIDFCoefficients(25, 0, 0, 19);

            MotorConfigurationType motorConfigurationType = outtake1.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            outtake1.setMotorType(motorConfigurationType);
            outtake2.setMotorType(motorConfigurationType);

            outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
            setPIDFCoefficients(outtake1, MOTOR_VELO_PID);
            setPIDFCoefficients(outtake2, MOTOR_VELO_PID);
        }
    }

    public void setOuttakePower(double power) {
        outtake1.setPower(-power);
        outtake2.setPower(-power);
    }

    public void setOuttakeVelocityTPS(double ticksPerSecond) {
        outtake1.setVelocity(ticksPerSecond);
        outtake2.setVelocity(ticksPerSecond);
    }

    public void printTelemetry(Telemetry telemetry) {
        double outtakeVelocity = outtake1.getVelocity();
        telemetry.addData("outtake ticks/sec", outtakeVelocity);
        telemetry.addData("outtake rev/min", (outtakeVelocity * 60) / 28);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
}

class Vision {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    Vision(HardwareMap hardwareMap) {

        AprilTagMetadata myAprilTagMetadata;
        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagLibrary myAprilTagLibrary;

        // Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

        // Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
        // Get the AprilTagLibrary for the current season.
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Create a new AprilTagMetdata object and assign it to a variable.
        //myAprilTagMetadata = new AprilTagMetadata(20, "please work i beg", 6.5, DistanceUnit.INCH);

        // Add a tag to the AprilTagLibrary.Builder.
        //myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);

        // Build the AprilTag library and assign it to a variable.
        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        // Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        // Set the tag library.
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

        // Build the AprilTag processor and assign it to a variable.
        aprilTag = myAprilTagProcessorBuilder.build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        }   // end method initAprilTag()

    public void detectObeliskBlocking(Drivetrain drivetrain, Telemetry telemetry) {
        ElapsedTime timer = new ElapsedTime();
        double maxSeconds = 100;
        double clampValue = 0.3;
        double tolerance = 0;

        double error = 676767;
        while ((error > tolerance && error < -tolerance) || timer.seconds() < maxSeconds) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            while (detections.isEmpty() || detections.get(0).metadata == null) {
                detections = aprilTag.getDetections();
                if (timer.seconds() > maxSeconds) {
                    break;
                }
            }

            double bearing = detections.get(0).ftcPose.bearing;

            error = bearing * 0.05;

            drivetrain.setDrivetrainPower(0, 0, clamp(error, -clampValue, clampValue));

            telemetry.addData("error", error);
            telemetry.addData("timer", timer.seconds());
            telemetry.update();
        }
    }

    static double clamp(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
    }
}

// class that handles the drum, intake, and flick
class Spindex {
    private Telemetry telemetry;

    private ServoImplEx drumServo;

    final double[] intakePositions = {0, 0.3826, 0.7831};
    final double[] outtakePositions = {0.5745, 0.95, 0.1823};

    DcMotor intake;

    DcMotor flick;

    NormalizedColorSensor intakeColorSensor;

    NormalizedColorSensor outtakeColorSensor;

    private String drumMode;

    int drumPosition;

    String[] ballStates = {"empty", "empty", "empty"};

    private ElapsedTime switchCooldownTimer;

    // time it takes to go from position 0 to position 1 on the drum servo
    // (setting too low will mean the sensor sees the same ball multiple times)
    final double switchCooldownConstant = 1.3;

    double switchCooldown = switchCooldownConstant;

    private ElapsedTime flickTimer;

    final double flickMinTime = 1; // min time that the robot will try to flick the ball up for

    /**
     * should run BEFORE waitForStart()
     * */
    Spindex(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        drumServo = hardwareMap.get(ServoImplEx.class, "drumServo");
        drumServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotor.Direction.REVERSE);

        flick = hardwareMap.get(DcMotor.class, "flick");

        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorLeft"); // TODO: rename this
        intakeColorSensor.setGain(15);

        //outtakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "idk lmao"); // TODO: config the sensor below the outtake

        drumMode = "intake";
        drumPosition = 0;
    }

    /**
     * should run AFTER waitForStart()
     * */
    public void init() {
        switchCooldownTimer = new ElapsedTime();
        flickTimer = new ElapsedTime(676767);
        updateDrumPosition();
    }

    /**
     * updates drum position based on drumMode and drumPosition
     * <p>
     * gets ran once at the end of update()
     * */
    private void updateDrumPosition() {
        if (drumInIntakeMode()) {
            drumServo.setPosition(intakePositions[drumPosition]);
        }
        else if (drumInOuttakeMode()) {
            drumServo.setPosition(outtakePositions[drumPosition]);
        }
    }

    /**
     * returns the physical position of the drum at a certain mode/position
     * */
    private double getDrumPositions(String drumMode, int drumPosition) {
        if (drumMode.equals("intake")) {
            return intakePositions[drumPosition];
        } else if (drumMode.equals("outtake")) {
            return outtakePositions[drumPosition];
        }
        return 676767;
    }

    public void incrementDrumPosition() {
        int newDrumPosition = drumPosition + 1;
        String newDrumState = drumMode;
        if (newDrumPosition >= 3) {
            newDrumPosition = 0;

            if (drumInIntakeMode()) {
                newDrumState = "outtake";
            } else if (drumInOuttakeMode()) {
                newDrumState = "intake";
            }
        }

        setDrumState(newDrumState, newDrumPosition);
    }

    public boolean drumInOuttakeMode() {
        return drumMode.equals("outtake");
    }

    public boolean drumInIntakeMode() {
        return drumMode.equals("intake");
    }

    public boolean drumIsEmpty() {
        return ballStates[0].equals("empty") && ballStates[1].equals("empty") && ballStates[2].equals("empty");
    }

    public void setDrumState(String newDrumMode) {
        if (!this.drumMode.equals(newDrumMode)) {
            switchCooldown = switchCooldownConstant * (Math.abs(getDrumPositions(newDrumMode, this.drumPosition) - getDrumPositions(this.drumMode, this.drumPosition)));
            switchCooldownTimer.reset();

            this.drumMode = newDrumMode;
        }

    }

    public void setDrumState(int newDrumPosition) {
        if (this.drumPosition != newDrumPosition) {
            switchCooldown = switchCooldownConstant * (Math.abs(getDrumPositions(this.drumMode, newDrumPosition) - getDrumPositions(this.drumMode, this.drumPosition)));
            switchCooldownTimer.reset();

            this.drumPosition = newDrumPosition;
        }

    }

    public void setDrumState(String newDrumMode, int newDrumPosition) {
        if ((!this.drumMode.equals(newDrumMode)) || (this.drumPosition != newDrumPosition)) {
            switchCooldown = switchCooldownConstant * (Math.abs(getDrumPositions(newDrumMode, newDrumPosition) - getDrumPositions(this.drumMode, this.drumPosition)));
            switchCooldownTimer.reset();

            this.drumMode = newDrumMode;
            this.drumPosition = newDrumPosition;
        }
    }

    private void setDrumStateToNextOuttake() {
        for (int i = 0; i < ballStates.length; i++) {
            String ballState = ballStates[i];
            if (!ballState.equals("empty")) {
                setDrumState("outtake", i);
                break;
            }
        }
    }

    private void setDrumStateToNextOuttake(String color) {
        for (int i = 0; i < ballStates.length; i++) {
            String ballState = ballStates[i];
            if (ballState.equals(color)) {
                setDrumState("outtake", i);
                break;
            }
        }
    }

    public boolean drumIsSwitching() {
        return switchCooldownTimer.seconds() < switchCooldown;
    }

    public boolean drumIsFlicking() {
        return flickTimer.seconds() < flickMinTime;
    }

    /**
     * returns true if a ball is detected
     * */
    private boolean detectBallIntake() {
        // if still in cooldown, return
        if (drumIsSwitching()) {
            return false;
        }

        // get distance from sensor
        boolean ballDetected = ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM) < 3;

        if (!ballDetected) { // could add && Objects.equals(ballStates[drumPosition], "empty") to this statement
            return false;
        }

        // get color from sensor
        NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        // assume a ball is purple unless proven green (most balls on the field are purple)
        String color = "purple";
        if (hsvValues[0] < 180) {
            color = "green";
        }

        ballStates[drumPosition] = color;

        return true;
    }

    /**
     * move the drum to the next outtake slot with a ball in it and flick it
     * */
    public void flickNextBall() {
        setDrumStateToNextOuttake();
        flickTimer.reset();
    }

    /**
     * move the drum to the next outtake slot with a ball of a certain color in it and flick it
     */
    public void flickNextBall(String color) {
        setDrumStateToNextOuttake(color);
        flickTimer.reset();
    }

    /**
     * should be called in the event loop
     * */
    public void update(boolean outputTelemetry) {
        if (drumInIntakeMode()) {
            if (detectBallIntake()) {
                incrementDrumPosition();
            }
        }

        if (drumInOuttakeMode() && drumIsFlicking()) {
            if (!drumIsSwitching()) {
                flick.setPower(1);
                ballStates[drumPosition] = "empty";
            } else {
                flickTimer.reset();
            }
        } else {
            flick.setPower(0);
        }

        if (drumIsEmpty() && drumInOuttakeMode() && !drumIsSwitching() && !drumIsFlicking()) {
            setDrumState("intake", 0);
        }

        updateDrumPosition();

        if (outputTelemetry) {
            telemetry.addData("ballStates[0]", ballStates[0]);
            telemetry.addData("ballStates[1]", ballStates[1]);
            telemetry.addData("ballStates[2]", ballStates[2]);
            telemetry.addData("drumMode", drumMode);
            telemetry.addData("drumPosition", drumPosition);
            telemetry.addData("switchCooldown", switchCooldown);
            telemetry.addData("switchCooldownTimer.seconds()", switchCooldownTimer.seconds());
        }
    }
}