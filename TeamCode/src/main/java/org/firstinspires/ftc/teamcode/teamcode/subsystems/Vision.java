package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.teamcode.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {
    private boolean initialized = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Localizer localizer;
    private final Vector2d goalPosition;
    private AprilTagDetection latestDetection;
    private Pose2d lastPose;
    private boolean faceGoalCalledThisLoop = false;
    private boolean faceGoalCalledLastLoop = false;
    private ElapsedTime faceGoalTimer = new ElapsedTime();
    private double faceGoalStartDistance;

    public int obeliskId = 22; // guess that the obelisk is 22 if we aren't able to detect it
    public double goalDistance = 0;
    public boolean canSeeGoalAprilTag = false;
    public boolean seenGoalAprilTag = false;

    public Servo cameraServo;

    private double cameraPitch = 22;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 7, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90 + cameraPitch, 0, 0);

    public Vision(HardwareMap hardwareMap, boolean isRedAlliance) {
        /// see ConceptAprilTagLocalization.java
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessor(aprilTag)
                .build();
        visionPortal.setProcessorEnabled(aprilTag, true);

        localizer = new PinpointLocalizer(hardwareMap, new Pose2d(PoseStorage.currentPose.component1(), PoseStorage.currentPose.component2()));
        PoseStorage.currentPose = new Pose2d(0, 0, 0);

        lastPose = localizer.getPose();

        if (isRedAlliance) {
            goalPosition = new Vector2d(-58, 55);
        } else {
            goalPosition = new Vector2d(-58, -55);
        }

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(0.84);
    }

    private void initCamera() {
//        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//        exposureControl.setMode(ExposureControl.Mode.Manual);
//        exposureControl.setExposure(5, TimeUnit.MILLISECONDS);
//
//        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//        gainControl.setGain(100);
    }

    public void detectGoalAprilTag() {
        canSeeGoalAprilTag = false;

        for (AprilTagDetection detection : aprilTag.getDetections()) {

            if ( ((detection.id == 24) || (detection.id == 20)) && detection.metadata != null ) {
                latestDetection = detection;
                double xPos = detection.robotPose.getPosition().x;
                double yPos = detection.robotPose.getPosition().y;
                double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) - (Math.PI / 2);
                localizer.setPose(new Pose2d(xPos, yPos, heading));

                canSeeGoalAprilTag = true;
                seenGoalAprilTag = true;
            }
        }
    }

    public double findPitchError() {
        double cameraPitchError = 0;

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if ( ((detection.id == 24) || (detection.id == 20)) && detection.metadata != null ) {
                cameraPitchError = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
            }
        }

        return cameraPitchError;
    }

//    private double getTargetRelativeBearing(double currentBearing) {
//        return targetAbsoluteBearing - currentBearing;
//    }

    /**
     * sets drivetrain powers to try to face the goal
     * */
    public void faceGoal(Drivetrain drivetrain, Telemetry telemetry) {
        faceGoalCalledThisLoop = true;

        double targetBearing = CustomMath.angleBetweenPoints(goalPosition, localizer.getPose().component1());
        double currentBearing = localizer.getPose().component2().toDouble();

        if (!faceGoalCalledLastLoop) {
            faceGoalTimer.reset();
            faceGoalStartDistance = targetBearing - currentBearing;
        }

        // use motion profiling to change targetBearing to a good value
        double maxAcceleration = Math.PI / 2; // TODO: tune ts
        double maxVelocity = Math.PI / 2;

        // only use motion profiling if the error is bigger than 30 degrees
        if (false){ //if (Math.abs(targetBearing - currentBearing) > Math.PI / 6) { // TODO: tune the proportional controller and then change this line back
            targetBearing = CustomMath.motionProfile(maxAcceleration, maxVelocity, faceGoalStartDistance, faceGoalTimer.seconds());
            telemetry.addLine("using motion profiling: true");
        } else {
            telemetry.addLine("using motion profiling: false");
        }


        double bearingError = targetBearing - currentBearing;
        double rotationPower = CustomMath.clamp(bearingError, -0.5, 0.5);
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
//        telemetry.addData("target absolute bearing", targetAbsoluteBearing);
        telemetry.addData("goal distance", goalDistance);

//        telemetry.addData("bearing", currentBearing);

        telemetry.addData("can see goal april tag", canSeeGoalAprilTag);

        telemetry.addData("camera pitch", cameraPitch);

        telemetry.addData("odo pose", "x: " + localizer.getPose().position.x + " y: " + localizer.getPose().position.y + " heading: " + localizer.getPose().heading.toDouble());
        if (canSeeGoalAprilTag) {
            telemetry.addData("april tag pose", "x: " + latestDetection.robotPose.getPosition().x + " y: " + latestDetection.robotPose.getPosition().y + " heading: " + latestDetection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
        }
    }

    public void update() {
        localizer.update();

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING && !initialized) {
            initCamera();
            initialized = true;
        }

        if (initialized) {
            detectGoalAprilTag();
        }

        goalDistance = CustomMath.distanceBetweenPoints(localizer.getPose().component1(), goalPosition) / 39.37;

        faceGoalCalledLastLoop = faceGoalCalledThisLoop;
        faceGoalCalledThisLoop = false;

        lastPose = localizer.getPose();
    }

    public Velocity getRequiredVelocity() {
        if (seenGoalAprilTag) {
            return TrajectoryMath.calculateBallLaunchVelocity(goalDistance);
        } else {
            return TrajectoryMath.calculateBallLaunchVelocity(1.2);
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
            boolean hardCoding = true;
            if (hardCoding) {
                if (distanceFromBucket < 1.5) {
                    return new Velocity(4.3, 55);
                } else {
                    return new Velocity(6.3, 45);
                }

            } else {
                double startSpeed = calculateStartSpeed(g, distanceFromBucket, h, theta);
                // these calculations actually work in reverse, so "endSpeed" and "endDirection" are actually the speed and direction that we need the ball to START with
                double endSpeed = calculateEndSpeed(startSpeed, g, distanceFromBucket, h, theta);
                double endDirection = calculateEndDirection(startSpeed, g, distanceFromBucket, h, theta);
                return new Velocity(endSpeed, -endDirection);
            }

        }
    }
}