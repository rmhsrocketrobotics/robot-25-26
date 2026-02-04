package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PDController;
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
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Drawing;
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
    public PinpointLocalizer localizer;
    private final Vector2d goalPosition;
    private AprilTagDetection latestDetection;
    private Pose2d lastPose;
    private boolean faceGoalCalledThisLoop = false;
    private boolean faceGoalCalledLastLoop = false;
    private ElapsedTime faceGoalTimer = new ElapsedTime();
    private double faceGoalStartDistance;
    private PDController xController;
    private PDController yController;
    private Vector2d faceGoalStartPosition;

    public int obeliskId = 22; // guess that the obelisk is 22 if we aren't able to detect it
    public double goalDistance = 0;
    public boolean canSeeGoalAprilTag = false;
    public boolean seenGoalAprilTag = false;

    public Servo cameraServo;

    private double cameraPitch = 10;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 7, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90 + cameraPitch, 0, 0);

    private boolean isRedAlliance;

    public Vision(HardwareMap hardwareMap, boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;

        /// see ConceptAprilTagLocalization.java
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
//                .setLensIntrinsics(1393.09, 1393.09, 974.952, 551.167)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
//                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(aprilTag)
                .build();
        visionPortal.setProcessorEnabled(aprilTag, true);

        localizer = new PinpointLocalizer(hardwareMap, new Pose2d(PoseStorage.currentPose.component1(), PoseStorage.currentPose.component2()));
        PoseStorage.currentPose = new Pose2d(0, 0, 0);

        lastPose = localizer.getPose();

        /// DON'T CHANGE THESE IF YOU WANT TO CHANGE THE AUTO TARGETING CHANGE THE NUMBERS IN faceGoal()
        if (isRedAlliance) {
            goalPosition = new Vector2d(-58, 55);
            //goalPosition = new Vector2d(-65, 60); possible new goal position???
        } else {
            goalPosition = new Vector2d(-58, -55);
        }

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");


        xController = new PDController(0.25, 0.03);
        yController = new PDController(0.25, 0.03);
    }

    public void init() {
        cameraServo.setPosition(0.5);
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
                double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + (Math.PI / 2);
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

        double targetBearing;
        if (isRedAlliance) {
            targetBearing = CustomMath.angleBetweenPoints(localizer.getPose().component1(), new Vector2d(-67, 69));
        } else {
            targetBearing = CustomMath.angleBetweenPoints(localizer.getPose().component1(), new Vector2d(-67, -69));
        }

//        if (goalDistance > 2.5) {
//            if (isRedAlliance) {
//                targetBearing = targetBearing + (Math.PI/12);
//            } else {
//                targetBearing = targetBearing - (Math.PI/12);
//            }
//        }

        double currentBearing = localizer.getPose().component2().toDouble();

        if (!faceGoalCalledLastLoop) {
            faceGoalTimer.reset();
            faceGoalStartDistance = targetBearing - currentBearing;

            faceGoalStartPosition = new Vector2d(localizer.driver.getPosX(DistanceUnit.INCH), localizer.driver.getPosY(DistanceUnit.INCH));

            xController.reset();
            yController.reset();
        }

//        // use motion profiling to change targetBearing to a good value
//        double maxAcceleration = Math.PI / 2; // TODO: tune ts
//        double maxVelocity = Math.PI / 2;
//
//        // only use motion profiling if the error is bigger than 30 degrees
//        if (false){ //if (Math.abs(targetBearing - currentBearing) > Math.PI / 6) { // TODO: tune the proportional controller and then change this line back
//            targetBearing = currentBearing + CustomMath.motionProfile(maxAcceleration, maxVelocity, faceGoalStartDistance, faceGoalTimer.seconds());
//            telemetry.addLine("using motion profiling: true");
//        } else {
//            telemetry.addLine("using motion profiling: false");
//        }

        double bearingError = targetBearing - currentBearing;
        if (Math.abs(bearingError + (2 * Math.PI)) < Math.abs(bearingError)) {
            bearingError = bearingError + (2 * Math.PI);
        } else if (Math.abs(bearingError - (2 * Math.PI)) < Math.abs(bearingError)) {
            bearingError = bearingError - (2 * Math.PI);
        }

        // find current position
        Vector2d currentPosition = new Vector2d(localizer.driver.getPosX(DistanceUnit.INCH), localizer.driver.getPosY(DistanceUnit.INCH));

        // calculate rotation power using a p controller
        double rotationPower = CustomMath.clamp(bearingError * 1.6, -0.7, 0.7);

        // calculate x and y power using a pd controller
        double xPower = xController.calculate(currentPosition.x, faceGoalStartPosition.x);
        xPower = CustomMath.clamp(xPower, -0.7, 0.7);

        double yPower = yController.calculate(currentPosition.y, faceGoalStartPosition.y);
        yPower = CustomMath.clamp(yPower, -0.7, 0.7);

        // rotate powers relative to robot
        Vector2d xyPower = CustomMath.rotatePointAroundOrigin(new Vector2d(xPower, yPower), -localizer.driver.getHeading(AngleUnit.RADIANS));

        // ok so when the pinpoint odo system says "X," they mean forward-backward, but when the drivetrain class says "X," it means left-right
        // that's why this is sus and android studio gives a warning
        drivetrain.setDrivetrainPower(xyPower.x, -xyPower.y, -rotationPower);

        // brake if facing about the correct angle
        if (Math.abs(xyPower.x) < 0.1 && Math.abs(xyPower.y) < 0.1 && Math.abs(rotationPower) < 0.1) {
            //drivetrain.isBraking = true;
        }
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
}