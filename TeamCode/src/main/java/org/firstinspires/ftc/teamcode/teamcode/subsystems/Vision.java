package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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
    public VisionPortal visionPortal;
    private double targetAbsoluteBearing = 0;
    public int obeliskId = 22; // guess that the obelisk is 22 if we aren't able to detect it
    private final boolean isRedAlliance;
    public double goalDistance = 0;
    public boolean seenGoalAprilTag = false;
    public Vision(HardwareMap hardwareMap, boolean isRedAlliance) {

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
        visionPortal.setProcessorEnabled(aprilTag, true);

        this.isRedAlliance = isRedAlliance;
    }

    public void init() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(5, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(100);
    }

    public void detectGoalAprilTag(double currentBearing) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if ( (detection.id == 24) || (detection.id == 20) ) {
                targetAbsoluteBearing = currentBearing + detection.ftcPose.bearing;

                double range = detection.ftcPose.range / 39.37;

                // TODO: figure out which one of these is correct (does ftcPose.range return actual distance or just distance on the x and y axis?? idk)
                goalDistance = range;
                //goalDistance = Math.sqrt((range * range) - (0.4572 * 0.4572)); // triangles

                seenGoalAprilTag = true;
            }
        }
    }

    private double getTargetRelativeBearing(double currentBearing) {
        return targetAbsoluteBearing - currentBearing;
    }

    /**
     * sets drivetrain powers to try to face the goal
     * */
    public void faceGoal(Drivetrain drivetrain, double currentBearing) {
        double bearingError = getTargetRelativeBearing(currentBearing);
        if (!isRedAlliance) {
            bearingError += 6;
        } else {
            bearingError -= 6;
        }

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
        telemetry.addData("goal distance", goalDistance);
    }

    public void update(double currentBearing) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING && !initialized) {
            init();
            initialized = true;
        }

        if (initialized) {
            detectGoalAprilTag(currentBearing);
        }
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