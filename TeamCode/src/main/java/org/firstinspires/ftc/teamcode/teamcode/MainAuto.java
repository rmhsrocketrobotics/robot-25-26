package org.firstinspires.ftc.teamcode.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.CustomMath;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.State;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Velocity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class MainAuto extends LinearOpMode {

    public boolean allianceIsRed() {
        return true;
    }

    public boolean useFarAuto() {
        return true;
    }

    public class RecordPose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            PoseStorage.currentPose = drive.localizer.getPose();
            return true;
        }
    }

    public Action recordPose() {
        return new RecordPose();
    }

    public class BallHandler { // combination of spindex and outtake
        Spindex spindex;
        Outtake outtake;
        AutoVision vision;

        public BallHandler(HardwareMap hardwareMap) {
            spindex = new Spindex(hardwareMap);
            spindex.ballStates = new Spindex.BallState[]{Spindex.BallState.GREEN, Spindex.BallState.PURPLE, Spindex.BallState.PURPLE};

            outtake = new Outtake(hardwareMap);
            //outtake.tolerance = 100;

            vision = new AutoVision(hardwareMap);
        }

        public void init() {
            spindex.init("outtake", 2);

            outtake.init();
        }

        public class TrackObelisk implements Action {
            final boolean trackObelisk;
            public TrackObelisk(boolean trackObelisk) {
                this.trackObelisk = trackObelisk;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (trackObelisk) {
                    vision.faceCameraToObelisk(drive.localizer.getPose());
                } else {
                    vision.cameraServo.setPosition(0.5);
                }

                vision.detectObelisk();
                telemetry.addData("obelisk id", vision.obeliskId);
                return true;
            }
        }

        public Action trackObelisk(boolean trackObelisk) {
            return new TrackObelisk(trackObelisk);
        }

        public class LaunchAllBalls implements Action {
            private boolean initialized = false;
            private double launchDistance;

            public LaunchAllBalls(double launchDistance) {
                this.launchDistance = launchDistance;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    if (vision.obeliskId == 21) {
                        spindex.queueBall("green");
                        spindex.queueBall("purple");
                        spindex.queueBall("purple");

                    } else if (vision.obeliskId == 22) {
                        spindex.queueBall("purple");
                        spindex.queueBall("green");
                        spindex.queueBall("purple");

                    } else if (vision.obeliskId == 23) {
                        spindex.queueBall("purple");
                        spindex.queueBall("purple");
                        spindex.queueBall("green");
                    }

                    outtake.setOuttakeVelocityAndHoodAngle(launchDistance);
                }

                spindex.update(outtake, State.OUTTAKE);
                outtake.update();

//                outtake.printTelemetry(telemetry);
//                spindex.printTelemetry(telemetry);

                telemetry.update();

                PoseStorage.ballStates = spindex.ballStates;

                return !spindex.shouldSwitchToIntake;
            }
        }
        public Action launchAllBalls(double launchDistance) {
            return new LaunchAllBalls(launchDistance);
        }

        public class RunActiveIntake implements Action {
            private boolean initialized = false;
            private boolean resetDrum;

            public RunActiveIntake(boolean resetDrum) {
                this.resetDrum = resetDrum;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    outtake.targetTicksPerSecond = 0;

                    if (this.resetDrum) {
                        spindex.recheckDrum();
                    }

                    spindex.intakeMotor.setPower(1);
                }
                spindex.update(outtake, State.INTAKE);
                outtake.update();

                PoseStorage.ballStates = spindex.ballStates;

                return true;
            }
        }
        public Action runActiveIntake(boolean resetDrum) {
            return new RunActiveIntake(resetDrum);
        }

        public class ReadyOuttake implements Action {
            private boolean initialized = false;
            private double launchDistance;
            private State currentState = State.INTAKE;

            public ReadyOuttake(double launchDistance) {
                this.launchDistance = launchDistance;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    spindex.update(outtake, currentState);

                    outtake.setOuttakeVelocityAndHoodAngle(launchDistance);
                }

                if (spindex.shouldSwitchToOuttake && currentState == State.INTAKE) {
                    currentState = State.OUTTAKE;
                }

                if (currentState == State.INTAKE) {
                    spindex.intakeMotor.setPower(1);

                } else if (currentState == State.OUTTAKE) {
                    spindex.intakeMotor.setPower(0);
                }

                spindex.update(outtake, currentState);
                outtake.update();

                PoseStorage.ballStates = spindex.ballStates;

                return true;
            }
        }
        public Action readyOuttake(double launchDistance) {
            return new ReadyOuttake(launchDistance);
        }
    }

    public final double pi = Math.PI;

    Pose2d beginPose;
    MecanumDrive drive;

    VelConstraint lowVelocity;
    TrajectoryActionBuilder startToLaunchZone;
    TrajectoryActionBuilder launchZoneToSecondBalls;
    TrajectoryActionBuilder secondBallsToLaunchZone;
    TrajectoryActionBuilder launchZoneToThirdBalls;
    TrajectoryActionBuilder thirdBallsToLaunchZone;
    TrajectoryActionBuilder launchZoneToEndPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        //Vision vision = new Vision(hardwareMap, allianceIsRed());
        BallHandler ballHandler = new BallHandler(hardwareMap);

        lowVelocity = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 10;
            }
        };

        // path actions: (.build() must be called in order for any of these to become actual actions)
        // IMPORTANT: for all of the following functions:
        // "first balls" are the PRELOAD;
        // "second balls" are the FIRST set of balls we PICK UP;
        // "third balls" are the SECOND set of balls we PICK UP
        // just moved all of the path building into functions but this still applies

        if (useFarAuto()) {
            farPath();
        } else {
            closePath();
        }

        double launchDistance;
        double finalLaunchDistance;
        if (useFarAuto()) {
            launchDistance = 2.75;
            finalLaunchDistance = launchDistance;
        } else {
            launchDistance = 0.9;
            finalLaunchDistance = 0.6;
        }
        Action firstBalls = new SequentialAction(new SleepAction(.3), ballHandler.launchAllBalls(launchDistance));

        // path actions combined with other actions
        Action launchFirstBalls = new ParallelAction(startToLaunchZone.build(), firstBalls);

        Action getSecondBalls = new RaceAction(launchZoneToSecondBalls.build(), ballHandler.runActiveIntake(true));
        Action returnToLaunchZoneWithSecondBalls = new RaceAction(secondBallsToLaunchZone.build(), ballHandler.readyOuttake(launchDistance));
//        Action returnToLaunchZoneWithSecondBalls = new RaceAction(secondBallsToLaunchZone.build(), ballHandler.runActiveIntake(false));
        Action launchSecondBalls = ballHandler.launchAllBalls(launchDistance);

        Action getThirdBalls = new RaceAction(launchZoneToThirdBalls.build(), ballHandler.runActiveIntake(true));
        Action returnToLaunchZoneWithThirdBalls = new RaceAction(thirdBallsToLaunchZone.build(), ballHandler.readyOuttake(finalLaunchDistance));
//        Action returnToLaunchZoneWithThirdBalls = new RaceAction(thirdBallsToLaunchZone.build(), ballHandler.runActiveIntake(false));
        Action launchThirdBalls = ballHandler.launchAllBalls(finalLaunchDistance);

        // combine all of the above actions into one big long sequential action
        Action runAutonomous = new SequentialAction(
                launchFirstBalls,
                getSecondBalls, returnToLaunchZoneWithSecondBalls, launchSecondBalls,
                getThirdBalls, returnToLaunchZoneWithThirdBalls, launchThirdBalls
        );

        // if this is a far auto, add the park pathing
        if (useFarAuto()) {
            runAutonomous = new SequentialAction(runAutonomous, launchZoneToEndPosition.build());
        }

        // add pose recording to the auto action
        runAutonomous = new RaceAction(runAutonomous, recordPose());

        // add auto camera tracking to the auto action
        if (useFarAuto()) {
            runAutonomous = new RaceAction(runAutonomous, ballHandler.trackObelisk(false));
        } else {
            runAutonomous = new RaceAction(runAutonomous, ballHandler.trackObelisk(true));
        }

        // this is in place of a waitForStart() call
//        waitForStart();
        while (opModeInInit() && !isStopRequested()) {
            ballHandler.vision.detectObelisk();
            telemetry.addData("obelisk", ballHandler.vision.obeliskId);
            telemetry.addLine("21 is gpp; 22 is pgp; 23 is ppg");
        }

//        ballHandler.obeliskId = 22;//vision.obeliskId;
        ballHandler.init();

        Actions.runBlocking(runAutonomous);
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }

    public void farPath() {
        // flip if on blue
        int flipConstant = 1;
        if (!allianceIsRed()) {
            flipConstant = -1;
        }

        beginPose = new Pose2d(61, 13.5 * flipConstant, Math.toRadians(180) * flipConstant);
        drive = new MecanumDrive(hardwareMap, beginPose);

        Vector2d launchPosition = new Vector2d(50.5, 14.5 * flipConstant);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-66, 60 * flipConstant));

        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(beginPose);

        startToLaunchZone = actionBuilder
                .splineTo(launchPosition, launchToGoalAngle);

        launchZoneToSecondBalls = startToLaunchZone.endTrajectory().fresh()
                .splineTo(new Vector2d(36, 28 * flipConstant), (pi/2) * flipConstant)
                .splineTo(new Vector2d(36, 61 * flipConstant), (pi/2) * flipConstant, lowVelocity); // slow mode

        secondBallsToLaunchZone = launchZoneToSecondBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(launchPosition, launchToGoalAngle - pi);

        launchZoneToThirdBalls = secondBallsToLaunchZone.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(13, 28 * flipConstant), (pi/2) * flipConstant)
                .splineTo(new Vector2d(13, 61 * flipConstant), (pi/2) * flipConstant, lowVelocity); // slow mode

        thirdBallsToLaunchZone = launchZoneToThirdBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(launchPosition, launchToGoalAngle - pi);

        launchZoneToEndPosition = thirdBallsToLaunchZone.endTrajectory().fresh() // only for far auto
                .setReversed(false)
                .splineTo(new Vector2d(20, 40 * flipConstant), pi);
    }

    public void closePath() {
        // flip if on blue
        int flipConstant = 1;
        if (!allianceIsRed()) {
            flipConstant = -1;
        }

        int ballPickupYPos = 27;

        beginPose = new Pose2d(-61, 36 * flipConstant, Math.toRadians(180) * flipConstant);
        drive = new MecanumDrive(hardwareMap, beginPose);

        Vector2d launchPosition = new Vector2d(-25, ballPickupYPos * flipConstant);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-58, 58 * flipConstant));

        Vector2d launchPositionFinal = new Vector2d(-50, ballPickupYPos * flipConstant);
        double launchToGoalAngleFinal = angleBetweenPoints(launchPositionFinal, new Vector2d(-65, 65 * flipConstant));

        Vector2d ball1PickupPosition = new Vector2d(-11, 55 * flipConstant);
        double ball1ToLaunchAngle = angleBetweenPoints(ball1PickupPosition, launchPosition);

        Vector2d ball2PickupPosition = new Vector2d(13, 61 * flipConstant);
        double ball2ToLaunchAngle = angleBetweenPoints(ball2PickupPosition, launchPosition);

        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(beginPose);

        startToLaunchZone = actionBuilder
                .setReversed(true)
                .splineTo(launchPosition, launchToGoalAngle - pi);

        launchZoneToSecondBalls = startToLaunchZone.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-11, (ballPickupYPos - 2.5) * flipConstant, (pi/2) * flipConstant), 0)
                .setTangent((pi/2) * flipConstant)
                .splineTo(ball1PickupPosition, (pi/2) * flipConstant, lowVelocity); // slow mode

        secondBallsToLaunchZone = launchZoneToSecondBalls.endTrajectory().fresh()
                .setTangent(ball1ToLaunchAngle)
                .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), ball1ToLaunchAngle);

        launchZoneToThirdBalls = secondBallsToLaunchZone.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(13, ballPickupYPos * flipConstant, (pi/2) * flipConstant), 0)
                .setTangent((pi/2) * flipConstant)
                .splineTo(ball2PickupPosition, (pi/2) * flipConstant, lowVelocity); // slow mode

        thirdBallsToLaunchZone = launchZoneToThirdBalls.endTrajectory().fresh()
                .setTangent((3*pi/2) * flipConstant)
                .splineToSplineHeading(new Pose2d(launchPositionFinal, launchToGoalAngleFinal), pi * flipConstant);
    }
}

class AutoVision {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public int obeliskId = 21;
    public Servo cameraServo;
    private final Vector2d obeliskPosition = new Vector2d(-72, 0);

    public AutoVision(HardwareMap hardwareMap) {
        // pretty sure that orientation doesn't matter for this
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessor(aprilTag)
                .build();
        visionPortal.setProcessorEnabled(aprilTag, true);

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
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

    public void faceCameraToObelisk(Pose2d cameraPose) {
        // uhhh this code probably isn't right TODO fix
        double angleToObelisk = CustomMath.angleBetweenPoints(cameraPose.position, obeliskPosition);
        double cameraAngle = angleToObelisk - cameraPose.heading.toDouble();

        cameraAngle = AngleUnit.normalizeRadians(cameraAngle);

        setCameraToAngle(cameraAngle);
    }

    /**
     * zero is looking straight ahead,
     * a positive angle is looking left,
     * and a negative angle is looking right
     * **/
    private void setCameraToAngle(double cameraAngle) {
        cameraAngle = -cameraAngle;

        double cameraMinAngle = -Math.PI / 2;
        double cameraMaxAngle = Math.PI / 2;
        double servoMinPosition = 0.18;
        double servoMaxPosition = 0.82;

        cameraAngle = CustomMath.clamp(cameraAngle, cameraMinAngle, cameraMaxAngle);
        double normalizedValue = (cameraAngle - cameraMinAngle) / (cameraMaxAngle - cameraMinAngle);
        double servoPosition = (normalizedValue * (servoMaxPosition - servoMinPosition)) + servoMinPosition;

        cameraServo.setPosition(servoPosition);
    }
}