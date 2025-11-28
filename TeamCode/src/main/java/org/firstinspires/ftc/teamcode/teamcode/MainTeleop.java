package org.firstinspires.ftc.teamcode.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import static java.lang.Math.sqrt;
import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;
import static java.lang.Math.toDegrees;
import static java.lang.Math.atan;

@TeleOp(group = "!main")

public class MainTeleop extends LinearOpMode{
    String state;
    Drivetrain drivetrain;
    Spindex spindex;
    Outtake outtake;
    Vision vision;
    Gamepad gamepad1Last;
    Gamepad gamepad2Last;

    @Override
    public void runOpMode() {
        state = "intake"; // states are: "intake", "transition", and "outtake"

        drivetrain = new Drivetrain(hardwareMap); // wheels
        spindex = new Spindex(hardwareMap); // drumServo, intake, flick
        outtake = new Outtake(hardwareMap); // outtake, hoodServo
        vision = new Vision(hardwareMap, true); // camera, pinpoint

        gamepad1Last = new Gamepad();
        gamepad2Last = new Gamepad();

//        //random ahh imu stuff dw
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        imu.initialize(parameters);
//        imu.resetYaw();

        telemetry.setMsTransmissionInterval(200); //default 250
        //telemetry.setNumDecimalPlaces(0, 5);

        //this is in place of a waitForStart() call
        while (opModeInInit()) {
            vision.detectObelisk();
        }

        spindex.init();

        while (opModeIsActive()) {
            // GAMEPAD 1 CODE:
            if (gamepad1.y && vision.seenGoalAprilTag) {
                vision.faceGoal(drivetrain);
            } else {
                if (gamepad1.left_bumper) {
                    // speed mode
                    drivetrain.setDrivetrainPower(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
                } else if (gamepad1.right_bumper) {
                    // slow mode
                    drivetrain.setDrivetrainPower(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x,
                            0.2, 0.2, 0.2);
                } else {
                    // normal mode
                    drivetrain.setDrivetrainPower(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x,
                            0.7, 0.7, 0.7);
                }
            }

            outtake.hoodServo.setPosition((gamepad2.left_stick_y / 2) + 0.5);
            telemetry.addData("servo position", outtake.hoodServo.getPosition());

            // literally all of the rest of this code is for gamepad 2:
            //spindex.flick.setPower(gamepad2.left_trigger);

//            if (gamepad2.y && !gamepad2Last.y) {
//                spindex.incrementDrumPosition();
//            }
//            if (gamepad2.x && !gamepad2Last.x) {
//                spindex.setDrumState("intake", 0);
//            }

            spindex.intake.setPower(gamepad2.right_trigger);

            // state specific code goes in these methods
            if (state.equals("intake")) {
                intakeMode();
            } else if (state.equals("outtake")) {
                outtakeMode();
            }

            gamepad2Last.copy(gamepad2);

            spindex.update();
            outtake.update();
            vision.update();

            //drivetrain.printTelemetry(telemetry);
            //outtake.printTelemetry(telemetry);
            //telemetry.addData("facing direction radians", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            //telemetry.addData("angular velocity radians", imu.getRobotAngularVelocity(AngleUnit.RADIANS));
            //telemetry.addData("facing direction degrees", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            //telemetry.addData("angular velocity degrees", imu.getRobotAngularVelocity(AngleUnit.DEGREES));
            vision.printTelemetry(telemetry);
            telemetry.addData("state", state);
            telemetry.update();
        }
    }

    public void intakeMode() {
        if ((gamepad2.dpad_up && !gamepad2Last.dpad_up) || spindex.shouldSwitchToOuttake) {
            state = "outtake";
            spindex.setDrumState("outtake", 0);
            //vision.seenGoalAprilTag = false;
            return;
        } else if (gamepad2.dpad_down && !gamepad2Last.dpad_down) {
            spindex.setDrumState("intake", 0);
            return;
        }
    }

    public void outtakeMode() {
        if ((gamepad2.dpad_down && !gamepad2Last.dpad_down) || spindex.shouldSwitchToIntake) {
            state = "intake";
            spindex.setDrumState("intake", 0);
            outtake.targetTicksPerSecond = 0;
            return;
        }

        //vision.detectGoalAprilTag();

        Velocity requiredVelocity = vision.getRequiredVelocity();
        telemetry.addData("target speed m/s", requiredVelocity.speed);
        telemetry.addData("target angle m/s", requiredVelocity.direction);
        //outtake.setOuttakeAndHoodToVelocity(requiredVelocity);

        if (gamepad2.left_bumper && !gamepad2Last.left_bumper) {
            spindex.flickNextBall("purple");
        }
        if (gamepad2.right_bumper && !gamepad2Last.right_bumper) {
            spindex.flickNextBall("green");
        }
    }
}

// little baby class used in the TrajectoryMath inner class
class Velocity {
    public double speed;
    public double direction;
    Velocity(double speed, double direction) {
        this.speed = speed;
        this.direction = direction;
    }
}

class CustomMath {
    static double clamp(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
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

    public void setDrivetrainPower(double yPower, double xPower, double rPower, double yPowerMultiplier, double xPowerMultiplier, double rPowerMultiplier) {
        yPower = yPower * yPowerMultiplier;
        xPower = xPower * xPowerMultiplier;
        rPower = rPower * rPowerMultiplier;

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
    Servo hoodServo;
    PIDFCoefficients MOTOR_VELO_PID;
    VoltageSensor batteryVoltageSensor;
    double targetTicksPerSecond;
    Outtake(HardwareMap hardwareMap) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo"); // TODO config this

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

        targetTicksPerSecond = 0;
    }

    public void setOuttakePower(double power) {
        outtake1.setPower(-power);
        outtake2.setPower(-power);
    }

    private void setOuttakeVelocityTPS(double ticksPerSecond) {
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

    public void update() {
        setOuttakeVelocityTPS(targetTicksPerSecond);
    }

    public void setOuttakeAndHoodToVelocity(Velocity velocity) {
        // change meters per second into ticks per second
        // UPDATE: this calculation is based on https://www.desmos.com/calculator/ye3y2vp7c2
        double metersPerSecond = CustomMath.clamp(velocity.speed, 4, 6);
        double ticksPerSecond = (262.7892 * metersPerSecond) - 64.05544;
        targetTicksPerSecond = ticksPerSecond;

        // TODO: finish this method (add hood support)
    }
}

class Vision {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private double targetAbsoluteBearing = 0;
    public int obeliskId = 22; // guess that the obelisk is 22 if we aren't able to detect it
    private final boolean isRedAlliance;
    GoBildaPinpointDriver pinpoint;
    double currentBearing = 0;
    private double goalDistance = 0;
    boolean seenGoalAprilTag = false;
    Vision(HardwareMap hardwareMap, boolean isRedAlliance) {

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

        this.isRedAlliance = isRedAlliance;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-155, 0, DistanceUnit.MM);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
        }

    public void detectGoalAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if ( (isRedAlliance && detection.id == 24) || (!isRedAlliance && detection.id == 20) ) {
                targetAbsoluteBearing = currentBearing + detection.ftcPose.bearing;

                double range = detection.ftcPose.range / 39.37;
                goalDistance = Math.sqrt((range * range) - (0.4572 * 0.4572)); // triangles

                seenGoalAprilTag = true;
            }
        }
    }

    private double getTargetRelativeBearing() {
        return targetAbsoluteBearing - currentBearing;
    }

    /**
     * sets drivetrain powers to try to face the goal
     * */
    public void faceGoal(Drivetrain drivetrain) {
        double bearingError = getTargetRelativeBearing();
        double rotationPower = CustomMath.clamp(bearingError * 0.03, -0.25, 0.25);
        drivetrain.setDrivetrainPower(0, 0, -rotationPower);
    }

    /**
     * returns true if the obelisk has successfully been detected
     * */
    public boolean detectObelisk() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
                obeliskId = detection.id;
                return true;
            }
        }
        return false;
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("obelisk id", obeliskId);
        telemetry.addData("target absolute bearing", targetAbsoluteBearing);
        telemetry.addData("bearing", currentBearing);
        telemetry.addData("goal distance", goalDistance);
    }

    public void update() {
        pinpoint.update();
        currentBearing = pinpoint.getHeading(AngleUnit.DEGREES);

        detectGoalAprilTag();
    }

    public Velocity getRequiredVelocity() {
        if (seenGoalAprilTag) {
            return TrajectoryMath.calculateBallLaunchVelocity(goalDistance);
        } else {
            return TrajectoryMath.calculateBallLaunchVelocity(2);
        }
    }

    // inner class :O (has functions for calculating the trajectory of the ball)
    static class TrajectoryMath {
        static final double g = 9.81; // force of gravity in meters per second squared
        static final double h = 0.7; // height of the target above the launch point in meters
        static final double theta = toRadians(30); // angle that we are trying to get the ball to enter the bucket with in degrees above the horizon

        private static double calculateStartSpeed(double g, double d, double h, double theta) {
            double numerator = g * d * d;
            double denominator = 2 * cos(theta) * cos(theta) * (d * tan(theta) + h);

            return sqrt(numerator / denominator);
        }

        private static double calculateEndSpeed(double startSpeed, double g, double d, double h, double theta) {
            double xSpeed = startSpeed * cos(theta);
            double ySpeed = (startSpeed * sin(theta)) - ((g * d) / xSpeed);

            return sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        }

        private static double calculateEndDirection(double startSpeed, double g, double d, double h, double theta) {
            double xSpeed = startSpeed * cos(theta);
            double ySpeed = (startSpeed * sin(theta)) - ((g * d) / xSpeed);

            return toDegrees(atan(ySpeed / xSpeed));
        }

        public static Velocity calculateBallLaunchVelocity(double distanceFromBucket) {
            double startSpeed = calculateStartSpeed(g, distanceFromBucket, h, theta);
            // these calculations actually work in reverse, so "endSpeed" and "endDirection" are actually the speed and direction that we need the ball to START with
            double endSpeed = calculateEndSpeed(startSpeed, g, distanceFromBucket, h, theta);
            double endDirection = calculateEndDirection(startSpeed, g, distanceFromBucket, h, theta);
            return new Velocity(endSpeed, endDirection);
        }
    }
}

// class that handles the drum, intake, and flick
class Spindex {
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

    public boolean shouldSwitchToIntake = false;
    public boolean shouldSwitchToOuttake = false;

    /**
     * should run BEFORE waitForStart()
     * */
    Spindex(HardwareMap hardwareMap) {
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
    public void update() {
        shouldSwitchToIntake = false;
        shouldSwitchToOuttake = false;

        if (drumInIntakeMode()) {
            if (detectBallIntake()) {
                incrementDrumPosition();
                if (drumInOuttakeMode()) {
                    shouldSwitchToOuttake = true;
                }
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
            shouldSwitchToIntake = true;
        }

        updateDrumPosition();
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("ballStates[0]", ballStates[0]);
        telemetry.addData("ballStates[1]", ballStates[1]);
        telemetry.addData("ballStates[2]", ballStates[2]);
        telemetry.addData("drumMode", drumMode);
        telemetry.addData("drumPosition", drumPosition);
        telemetry.addData("switchCooldown", switchCooldown);
        telemetry.addData("switchCooldownTimer.seconds()", switchCooldownTimer.seconds());
    }
}
