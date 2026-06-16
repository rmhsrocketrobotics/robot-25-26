package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.teamcode.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Vision {
    private boolean initialized = false;
    private Limelight3A limelight;

    public PinpointLocalizer localizer;
    private final Vector2d goalPosition;
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

    /// can you see the april tag RIGHT NOW
    public boolean canSeeGoalAprilTag = false;

    /// have we EVER seen the april tag
    public boolean seenGoalAprilTag = false;

    public Servo cameraServo;

    private boolean isRedAlliance;

    public Vision(HardwareMap hardwareMap, boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        localizer = new PinpointLocalizer(hardwareMap, new Pose2d(PoseStorage.currentPose.component1(), PoseStorage.currentPose.component2()));
        if (PoseStorage.currentPose.component1().x != 0) { // TODO see if ts works
            seenGoalAprilTag = true;
        }
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

    public void detectAprilTag() {
        canSeeGoalAprilTag = false;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D robotPose = result.getBotpose();

            double xPos = robotPose.getPosition().x;
            double yPos = robotPose.getPosition().y;
            double heading = robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + (Math.PI / 2);

            localizer.setPose(new Pose2d(xPos, yPos, heading));

            canSeeGoalAprilTag = true;
            seenGoalAprilTag = true;
        }
    }

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
//                targetBearing = targetBearing + (Math.PI/24);
//            } else {
//                targetBearing = targetBearing - (Math.PI/24);
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

        telemetry.addData("odo pose", "x: " + localizer.getPose().position.x + " y: " + localizer.getPose().position.y + " heading: " + localizer.getPose().heading.toDouble());

        // limelight telemetry data below: VVVVVVVVVVVVVV

        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
    }

    public void update() {
        localizer.update();
        detectAprilTag();

        goalDistance = CustomMath.distanceBetweenPoints(localizer.getPose().component1(), goalPosition) / 39.37;

        faceGoalCalledLastLoop = faceGoalCalledThisLoop;
        faceGoalCalledThisLoop = false;

        lastPose = localizer.getPose();
    }
}