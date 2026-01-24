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

    public class ClosePathGenerator {
        int flipConstant;

        Vector2d launchPosition;
        double launchToGoalAngle;
        Pose2d launchPose;

        Pose2d classifierPose;

        ClosePathGenerator() {
            if (allianceIsRed()) {
                flipConstant = 1;
            } else {
                flipConstant = -1;
            }

            beginPose = new Pose2d(-48, 52 * flipConstant, Math.toRadians(130) * flipConstant);
            drive = new MecanumDrive(hardwareMap, beginPose);

            launchPosition = new Vector2d(-11, 20 * flipConstant);
            launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-58, 58 * flipConstant));
            launchPose = new Pose2d(launchPosition, launchToGoalAngle);

            classifierPose = new Pose2d(13,58 * flipConstant, 3*pi/4 * flipConstant);
        }

        public Action startToMiddleBalls() {
            return drive.actionBuilder(beginPose)
                    // back up while facing the goal
                    .setReversed(true)
                    .splineTo(new Vector2d(-23, 24 * flipConstant), launchToGoalAngle - pi, mediumVelocity) // slowed

                    // go to middle row of balls
                    .splineToSplineHeading(new Pose2d(13, 30 * flipConstant, (pi/2) * flipConstant), (pi/2) * flipConstant)
                    .splineTo(new Vector2d(13, 61 * flipConstant), (pi/2) * flipConstant, lowVelocity) // slowed

                    .build();
        }

        public Action toLaunchZone(Pose2d startPose) {
            return drive.actionBuilder(startPose)
                    .setTangent((3*pi/2) * flipConstant)
                    .splineToSplineHeading(launchPose, pi)

                    .build();
        }

        public Action toFinalLaunchZone(Pose2d startPose) {
            Vector2d endLaunchPosition = new Vector2d(-46, 25 * flipConstant);
            double endLaunchToGoalAngle = angleBetweenPoints(endLaunchPosition, new Vector2d(-58, 58 * flipConstant));
            double startToFinalLaunchAngle = angleBetweenPoints(startPose.component1(), endLaunchPosition);

            return drive.actionBuilder(startPose)
                    .setTangent(startToFinalLaunchAngle)
                    .splineToSplineHeading(new Pose2d(endLaunchPosition, endLaunchToGoalAngle), startToFinalLaunchAngle)

                    .build();
        }

        public Action classifierToLaunchZone() {
            return drive.actionBuilder(classifierPose)
                    .setTangent((3*pi/2) * flipConstant)
                    .splineToSplineHeading(launchPose, pi)

                    .build();
        }

        public Action launchZoneToClassifier() {
            return drive.actionBuilder(launchPose)
                    .setTangent((pi/4) * flipConstant)
                    .splineToSplineHeading(classifierPose, (pi/2) * flipConstant)

                    .build();
        }

        public Action launchZoneToCloseBalls() {
            return drive.actionBuilder(launchPose)
                    .setTangent((pi/2) * flipConstant)
                    .splineToSplineHeading(new Pose2d(-11, 30 * flipConstant, (pi/2) * flipConstant), (pi/2) * flipConstant)
                    .splineTo(new Vector2d(-11, 55 * flipConstant), (pi/2) * flipConstant, lowVelocity) // slowed

                    .build();
        }

        public class UpdateLocalizer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drive.localizer.update();
                return true;
            }
        }

        public Action updateLocalizer() {
            return new UpdateLocalizer();
        }
    }

    public class FarPathGenerator {
        int flipConstant;

        Vector2d launchPosition;
        double launchToGoalAngle;
        Pose2d launchPose;

        FarPathGenerator() {
            if (allianceIsRed()) {
                flipConstant = 1;
            } else {
                flipConstant = -1;
            }

            beginPose = new Pose2d(61, 13.5 * flipConstant, Math.toRadians(180) * flipConstant);
            drive = new MecanumDrive(hardwareMap, beginPose);

            launchPosition = new Vector2d(50.5, 14.5 * flipConstant);
            launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-66, 60 * flipConstant));
            launchPose = new Pose2d(launchPosition, launchToGoalAngle);
        }

        public Action startToLaunchZone() {
            return drive.actionBuilder(beginPose)
                    .splineTo(launchPosition, launchToGoalAngle)

                    .build();
        }

        public Action toLaunchZone(Pose2d startPose) {
            return drive.actionBuilder(startPose)
                    .setReversed(true)
                    .splineTo(launchPosition, launchToGoalAngle - pi)

                    .build();
        }

        public Action launchZoneToFarBalls() {
            return drive.actionBuilder(launchPose)
                    .setReversed(false)
                    .splineTo(new Vector2d(36, 28 * flipConstant), (pi/2) * flipConstant)
                    .splineTo(new Vector2d(36, 61 * flipConstant), (pi/2) * flipConstant, lowVelocity) // slowed

                    .build();
        }

        public Action launchZoneToMiddleBalls() {
            return drive.actionBuilder(launchPose)
                    .setReversed(false)
                    .splineTo(new Vector2d(13, 28 * flipConstant), (pi/2) * flipConstant)
                    .splineTo(new Vector2d(13, 61 * flipConstant), (pi/2) * flipConstant, lowVelocity) // slowed

                    .build();
        }

        public Action launchZoneToPark() {
            return drive.actionBuilder(launchPose)
                    .setReversed(false)
                    .setTangent(pi/2 * flipConstant)
                    .splineToSplineHeading(new Pose2d(50.5, 35 * flipConstant, pi), pi/2 * flipConstant)

                    .build();
        }

        public Action launchZoneToHumanBalls() {
            return drive.actionBuilder(launchPose)
                    .setTangent(pi/2)
                    .splineToSplineHeading(new Pose2d(56, 30 * flipConstant, pi/2 * flipConstant), pi/2 * flipConstant)
                    .splineToConstantHeading(new Vector2d(61, 55 * flipConstant), pi/2 * flipConstant)
                    .splineToConstantHeading(new Vector2d(61, 60 * flipConstant), pi/2 * flipConstant, lowVelocity) // slowed

                    .build();
        }

        public Action humanBallsToLaunchZone() {
            return drive.actionBuilder(new Pose2d(61, 60 * flipConstant, pi/2 * flipConstant)) /// this should be the human ball pose
                    .setTangent(3*pi/2)
                    .splineToConstantHeading(new Vector2d(56, 30 * flipConstant), 3*pi/2 * flipConstant)
                    .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), 3*pi/2 * flipConstant)
                    .setTangent(launchToGoalAngle)

                    .build();
        }

        public Action launchZoneToSweepDrew() {
            return drive.actionBuilder(launchPose)
                    .setTangent(pi/2 * flipConstant)
                    .splineToSplineHeading(new Pose2d(59, 35 * flipConstant, pi/2 * flipConstant), pi/2 * flipConstant)
                    .splineToSplineHeading(new Pose2d(40, 60 * flipConstant, pi), pi)
                    .splineToSplineHeading(new Pose2d(20, 60 * flipConstant, pi), pi)

                    .build();
        }

        public Action launchZoneToSweepJonah() {
            Vector2d sweepPosition = new Vector2d(58, 37 * flipConstant);
            double launchToSweepAngle = angleBetweenPoints(launchPosition, sweepPosition);

            return drive.actionBuilder(launchPose)
                    .setTangent(pi/2 * flipConstant)
                    .splineToSplineHeading(new Pose2d(sweepPosition, pi/2 * flipConstant), launchToSweepAngle)
                    .splineToSplineHeading(new Pose2d(54, 52 * flipConstant, 3*pi/4 * flipConstant), 5*pi/6 * flipConstant)
                    .splineToSplineHeading(new Pose2d(20, 60 * flipConstant, pi), pi)

                    .build();
        }

        public class UpdateLocalizer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drive.localizer.update();
                return true;
            }
        }

        public Action updateLocalizer() {
            return new UpdateLocalizer();
        }
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
            private boolean autoFindLaunchDistance;
            private Vector2d goalPosition;
            private boolean sortBalls;

            public LaunchAllBalls(double launchDistance, boolean sortBalls) {
                this.sortBalls = sortBalls;

                spindex.intakeMotor.setPower(0);

                if (launchDistance == 0) {
                    this.autoFindLaunchDistance = true;
                    if (allianceIsRed()) {
                        goalPosition = new Vector2d(-58, 55);
                    } else {
                        goalPosition = new Vector2d(-58, -55);
                    }
                    this.launchDistance = CustomMath.distanceBetweenPoints(drive.localizer.getPose().component1(), goalPosition) / 39.37;;

                } else {
                    this.autoFindLaunchDistance = false;
                    this.launchDistance = launchDistance;
                }
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    if (!sortBalls) {
                        spindex.shootAllBalls();

                    } else if (vision.obeliskId == 21) {
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

                    } else {
                        spindex.shootAllBalls();
                    }

                    outtake.setOuttakeVelocityAndHoodAngle(launchDistance);
                }

                if (autoFindLaunchDistance) {
                    this.launchDistance = CustomMath.distanceBetweenPoints(drive.localizer.getPose().component1(), goalPosition) / 39.37;
                    // alr bro listen ik this is a stupid way to try to do moving while shooting but i can't be bothered and it might work
                    // todo fix if it doesn't work tho
                    outtake.setOuttakeVelocityAndHoodAngle(launchDistance + 0.2);
                }

                spindex.update(outtake, State.OUTTAKE);
                outtake.update();

//                outtake.printTelemetry(telemetry);
//                spindex.printTelemetry(telemetry);
//                telemetry.update();

                PoseStorage.ballStates = spindex.ballStates;

                return !spindex.shouldSwitchToIntake;
            }
        }
        public Action launchAllBalls(double launchDistance, boolean sortBalls) {
            return new LaunchAllBalls(launchDistance, sortBalls);
        }

        public Action launchAllBalls(double launchDistance) {
            return new LaunchAllBalls(launchDistance, true);
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

                return !spindex.shouldSwitchToOuttake;
            }
        }
        public Action runActiveIntake(boolean resetDrum) {
            return new RunActiveIntake(resetDrum);
        }

        public class ReadyOuttake implements Action {
            private boolean initialized = false;
            private double launchDistance;
            private boolean sortBalls;

            public ReadyOuttake(double launchDistance, boolean sortBalls) {
                this.launchDistance = launchDistance;
                this.sortBalls = sortBalls;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    spindex.intakeMotor.setPower(1);

                    if (!sortBalls) {
                        spindex.setDrumStateToNextOuttake();

                    } else if (vision.obeliskId == 21) {
                        spindex.setDrumStateToNextOuttake(Spindex.BallState.GREEN, true);

                    } else if (vision.obeliskId == 22 || vision.obeliskId == 23) {
                        spindex.setDrumStateToNextOuttake(Spindex.BallState.PURPLE, true);

                    } else {
                        spindex.setDrumStateToNextOuttake();
                    }

                    outtake.setOuttakeVelocityAndHoodAngle(launchDistance);
                }

                spindex.update(outtake, State.OUTTAKE);
                outtake.update();

                PoseStorage.ballStates = spindex.ballStates;

                return true;
            }
        }

        public Action readyOuttake(double launchDistance, boolean sortBalls) {
            return new ReadyOuttake(launchDistance, sortBalls);
        }

        public Action readyOuttake(double launchDistance) {
            return new ReadyOuttake(launchDistance, true);
        }
    }

    public final double pi = Math.PI;

    Pose2d beginPose;
    MecanumDrive drive;

    VelConstraint lowVelocity;
    VelConstraint mediumVelocity;
    TrajectoryActionBuilder startToLaunchZone;
    TrajectoryActionBuilder launchZoneToSecondBalls;
    TrajectoryActionBuilder secondBallsToLaunchZone;
    TrajectoryActionBuilder launchZoneToThirdBalls;
    TrajectoryActionBuilder thirdBallsToLaunchZone;
    TrajectoryActionBuilder launchZoneToEndPosition;
    BallHandler ballHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        //Vision vision = new Vision(hardwareMap, allianceIsRed());
        ballHandler = new BallHandler(hardwareMap);

        lowVelocity = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 10;
            }
        };

        mediumVelocity = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 15;
            }
        };

        if (useFarAuto()) {
            runFarAutoBlocking();
        } else {
            runCloseAutoBlocking();
        }
    }

    public void runFarAutoBlocking() {
        // TODO add pose recording to this
        FarPathGenerator path = new FarPathGenerator();

        Action autonomous1 = new SequentialAction(
                // launch preload
                new ParallelAction(path.startToLaunchZone(), ballHandler.launchAllBalls(3, true)),

                // get human balls
                new RaceAction(
                        new SequentialAction(path.launchZoneToHumanBalls(), new SleepAction(0.5)),
                        ballHandler.runActiveIntake(true)
                ),

                // go back to launch zone
                new RaceAction(path.humanBallsToLaunchZone(), ballHandler.readyOuttake(3, true)),

                // fire human balls
                ballHandler.launchAllBalls(3, true),

                // get far balls
                new RaceAction(path.launchZoneToFarBalls(), ballHandler.runActiveIntake(true))
        );

        ballHandler.init();

        while (opModeInInit() && !isStopRequested()) {
            ballHandler.vision.detectObelisk();
            telemetry.addData("obelisk", ballHandler.vision.obeliskId);
            telemetry.addLine("21 is gpp; 22 is pgp; 23 is ppg");
            telemetry.update();
        }

        Actions.runBlocking(autonomous1);

        Action autonomous2 = new SequentialAction(
                // fire far balls
                new RaceAction(path.toLaunchZone(drive.localizer.getPose()), ballHandler.readyOuttake(3, true)),

                // get middle balls
                new RaceAction(path.launchZoneToMiddleBalls(), ballHandler.runActiveIntake(true))
        );

        Actions.runBlocking(autonomous2);

        Action autonomous3 = new SequentialAction(
                // fire middle balls
                new RaceAction(path.toLaunchZone(drive.localizer.getPose()), ballHandler.readyOuttake(3, true)),

                // park
                new RaceAction(path.launchZoneToPark(), ballHandler.runActiveIntake(true))
        );

        Actions.runBlocking(autonomous3);
    }

    public void runCloseAutoBlocking() {
        ClosePathGenerator path = new ClosePathGenerator();

        // shoots preload then intakes middle balls
        Action shootAndIntakeMiddleBalls = new SequentialAction(ballHandler.launchAllBalls(0, false), ballHandler.runActiveIntake(true));

        // add path to the action
        shootAndIntakeMiddleBalls = new RaceAction(shootAndIntakeMiddleBalls, path.startToMiddleBalls());

        // add auto camera tracking to the auto action
        shootAndIntakeMiddleBalls = new RaceAction(shootAndIntakeMiddleBalls, ballHandler.trackObelisk(true));

        ballHandler.init();

        waitForStart();

        /// BLOCKING
        Actions.runBlocking(shootAndIntakeMiddleBalls);

        // shoot middle balls
        Action shootMiddleBalls = new RaceAction(ballHandler.readyOuttake(1.5, false), path.toLaunchZone(drive.localizer.getPose()));
        shootMiddleBalls = new SequentialAction(
                shootMiddleBalls,
                new RaceAction(ballHandler.launchAllBalls(1.5, false), path.updateLocalizer())
        );

        // go clear classifier
        Action clearClassifier = new RaceAction(
                ballHandler.runActiveIntake(true),
                new SequentialAction(path.launchZoneToClassifier(), path.updateLocalizer()),
                new SleepAction(8)
        );

        // shoot classifier balls
        Action shootClassifierBalls = new RaceAction(ballHandler.readyOuttake(1.5), path.classifierToLaunchZone());
        shootClassifierBalls = new SequentialAction(
                shootClassifierBalls,
                new RaceAction(ballHandler.launchAllBalls(1.5), path.updateLocalizer())
        );

        // intakes close balls
        Action intakeCloseBalls = new RaceAction(ballHandler.runActiveIntake(true), path.launchZoneToCloseBalls());

        /// BLOCKING
        Actions.runBlocking(new SequentialAction(shootMiddleBalls, clearClassifier, shootClassifierBalls, intakeCloseBalls));

        // shoot close balls
        Action shootCloseBalls = new RaceAction(ballHandler.readyOuttake(0.7), path.toFinalLaunchZone(drive.localizer.getPose()));
        shootCloseBalls = new SequentialAction(shootCloseBalls, ballHandler.launchAllBalls(0.7));

        // add pose recording
        shootCloseBalls = new RaceAction(shootCloseBalls, recordPose());

        /// BLOCKING (final)
        Actions.runBlocking(shootCloseBalls);
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }
}

class AutoVision {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public int obeliskId = 0;
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

        cameraServo.setPosition(0.5);
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