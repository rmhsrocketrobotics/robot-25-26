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

        Vision vision = new Vision(hardwareMap);

        Gamepad gamepad1Last = new Gamepad();
        Gamepad gamepad2Last = new Gamepad();

        //random ahh imu stuff dw
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        //telemetry.setMsTransmissionInterval(250); //default 250
        //telemetry.setNumDecimalPlaces(0, 5);

        waitForStart();

        spindex.init();
        //vision.detectObeliskBlocking(drivetrain, telemetry);

        while (opModeIsActive()) {
            // GAMEPAD 1 CODE:
            if (gamepad1.y && !gamepad1Last.y) {
                telemetry.speak("test test 1 2 3");
            }
            gamepad1Last.copy(gamepad1);

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
                spindex.setDrumPosition("intake", 0);
            }
            if (gamepad2.left_bumper && !gamepad2Last.left_bumper) {
                spindex.setDrumPositionToOuttakeColor("purple");
            }
            if (gamepad2.right_bumper && !gamepad2Last.right_bumper) {
                spindex.setDrumPositionToOuttakeColor("green");
            }

            gamepad2Last.copy(gamepad2);

            spindex.update(true);
            // GAMEPAD 2 CODE END

            drivetrain.printTelemetry(telemetry);
            outtake.printTelemetry(telemetry);
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
        myAprilTagMetadata = new AprilTagMetadata(20, "please work i beg", 6.5, DistanceUnit.INCH);

        // Add a tag to the AprilTagLibrary.Builder.
        myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);

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
    Telemetry telemetry;
    ServoImplEx drumServo;
    DcMotor intake;
    DcMotor flick;
    NormalizedColorSensor colorSensor;
    String drumMode;
    int drumPosition;
    String[] ballStates = {"none", "none", "none"};
    ElapsedTime switchCooldownTimer;
    final double switchCooldown = 1; // time before we can intake another ball (setting too low will mean the sensor sees the same ball multiple times)

    Spindex(HardwareMap hardwareMap, Telemetry telemetry) { // should run BEFORE waitForStart()
        this.telemetry = telemetry;

        drumServo = hardwareMap.get(ServoImplEx.class, "drumServo");
        drumServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotor.Direction.REVERSE);

        flick = hardwareMap.get(DcMotor.class, "flick");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorLeft");

        colorSensor.setGain(15);

        drumMode = "intake";
        drumPosition = 0;
    }

    public void init() { // should run AFTER waitForStart()
        switchCooldownTimer = new ElapsedTime();
        updateDrumPosition();
    }

    private void updateDrumPosition() { // updates drum position based on drumMode and drumPosition
        if (Objects.equals(drumMode, "intake")) {
            double[] positions = {0, 0.3826, 0.7831};
            drumServo.setPosition(positions[drumPosition]);
        }
        else if (Objects.equals(drumMode, "outtake")) {
            double[] positions = {0.5745, 0.95, 0.1823};
            drumServo.setPosition(positions[drumPosition]);
        }
    }

    public void incrementDrumPosition() {
        drumPosition += 1;
        if (drumPosition >= 3) {
            drumPosition = 0;

            if (Objects.equals(drumMode, "intake")) {
                drumMode = "outtake";
            } else if (Objects.equals(drumMode, "outtake")) {
                drumMode = "intake";
            }
        }
        updateDrumPosition();
    }

    public void setDrumPosition(String drumMode, int drumPosition) {
        this.drumMode = drumMode;
        this.drumPosition = drumPosition;
        updateDrumPosition();
    }

    public void setDrumPositionToOuttakeColor(String color) {
        for (int i = 0; i < ballStates.length; i++) {
            String ballState = ballStates[i];
            if (Objects.equals(ballState, color)) {
                drumMode = "outtake";
                drumPosition = i;
                break;
            }
        }
        updateDrumPosition();
    }

    private boolean detectBall() { // returns true if a ball is detected
        // if still in cooldown, return
        if (switchCooldownTimer.seconds() < switchCooldown) {
            return false;
        }

        // get distance from sensor
        boolean ballDetected = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 3;

        if (!ballDetected) { // could add && Objects.equals(ballStates[drumPosition], "none") to this statement
            return false;
        }

        // get color from sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        // assume a ball is purple unless proven green (most balls on the field are purple)
        String color = "purple";
        if (hsvValues[0] < 180) {
            color = "green";
        }

        ballStates[drumPosition] = color;

        switchCooldownTimer.reset();
        return true;
    }

    public void update(boolean outputTelemetry) { // should be called in the event loop
        if (Objects.equals(drumMode, "intake")) {
            if (detectBall()) {
                incrementDrumPosition();
            }

        }

        if (outputTelemetry) {
            telemetry.addData("ballStates0", ballStates[0]);
            telemetry.addData("ballStates1", ballStates[1]);
            telemetry.addData("ballStates2", ballStates[2]);
            telemetry.addData("drumMode", drumMode);
            telemetry.addData("drumPosition", drumPosition);
        }
    }
}