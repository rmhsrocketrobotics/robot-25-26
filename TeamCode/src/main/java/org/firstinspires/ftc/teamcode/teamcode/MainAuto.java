package org.firstinspires.ftc.teamcode.teamcode;

import android.util.Size;

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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public abstract class DynamicTrajectoryAction implements Action {
        private boolean initialized = false;
        private Action trajectoryAction;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                trajectoryAction = createPath();
                initialized = true;
            }

            return trajectoryAction.run(packet);
        }

        public abstract Action createPath();
    }

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

            classifierPose = new Pose2d(12,59 * flipConstant, 5*pi/8 * flipConstant);
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

        public class ToLaunchZone extends DynamicTrajectoryAction {
            @Override
            public Action createPath() {
                return drive.actionBuilder(drive.localizer.getPose())
                        .setTangent((3*pi/2) * flipConstant)
                        .splineToSplineHeading(launchPose, pi)

                        .build();
            }
        }

        public Action toLaunchZone() {
            return new ToLaunchZone();
        }

        public Action toLaunchZone(Pose2d startPose) {
            return drive.actionBuilder(startPose)
                    .setTangent((3*pi/2) * flipConstant)
                    .splineToSplineHeading(launchPose, pi)

                    .build();
        }

        public class ToFinalLaunchZone extends DynamicTrajectoryAction {
            @Override
            public Action createPath() {
                Pose2d startPose = drive.localizer.getPose();

                Vector2d endLaunchPosition = new Vector2d(-46, 25 * flipConstant);
                double endLaunchToGoalAngle = angleBetweenPoints(endLaunchPosition, new Vector2d(-58, 58 * flipConstant));
                double startToFinalLaunchAngle = angleBetweenPoints(startPose.component1(), endLaunchPosition);

                return drive.actionBuilder(startPose)
                        .setTangent(startToFinalLaunchAngle)
                        .splineToSplineHeading(new Pose2d(endLaunchPosition, endLaunchToGoalAngle), startToFinalLaunchAngle)

                        .build();
            }
        }

        public Action toFinalLaunchZone() {
            return new ToFinalLaunchZone();
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

        public class ToLaunchZone extends DynamicTrajectoryAction {
            @Override
            public Action createPath() {
                return drive.actionBuilder(drive.localizer.getPose())
                        .setReversed(true)
                        .splineTo(launchPosition, launchToGoalAngle - pi)

                        .build();
            }
        }

        public Action toLaunchZone() {
            return new ToLaunchZone();
        }

        public Action toLaunchZone(Pose2d startPose) {
            return drive.actionBuilder(startPose)
                    .setReversed(true)
                    .splineTo(launchPosition, launchToGoalAngle - pi)

                    .build();
        }

        public class ToLaunchZoneStraight extends DynamicTrajectoryAction {
            @Override
            public Action createPath() {
                Pose2d startPose = drive.localizer.getPose();

                double startToLaunchAngle = angleBetweenPoints(startPose.component1(), launchPosition);

                return drive.actionBuilder(startPose)
                        .setTangent(startToLaunchAngle)
                        .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), startToLaunchAngle)

                        .build();
            }
        }

        public Action toLaunchZoneStraight() {
            return new ToLaunchZoneStraight();
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

        public class ToPark extends DynamicTrajectoryAction {
            @Override
            public Action createPath() {
                Pose2d startPose = drive.localizer.getPose();
                Vector2d parkPosition = new Vector2d(56, 35 * flipConstant);

                double startToParkAngle = angleBetweenPoints(startPose.component1(), parkPosition);

                return drive.actionBuilder(startPose)
                        .setTangent(startToParkAngle)
                        .splineToConstantHeading(parkPosition, startToParkAngle)

                        .build();
            }
        }

        public Action toPark() {
            return new ToPark();
        }

        public Action launchZoneToHumanBalls() {
            return drive.actionBuilder(launchPose)
                    .setTangent(pi/2 * flipConstant)
                    .splineToSplineHeading(new Pose2d(53,60 * flipConstant, 3*pi/8 * flipConstant), pi/2 * flipConstant, lowVelocity) // slowed

                    .setTangent(0)
                    .splineToConstantHeading(new Vector2d(59, 60 * flipConstant), 0, lowVelocity)

                    .build();

//            return drive.actionBuilder(launchPose)
//                    .setTangent(pi/2 * flipConstant)
//                    .splineToSplineHeading(new Pose2d(56, 30 * flipConstant, pi/2 * flipConstant), pi/2 * flipConstant)
//                    .splineToConstantHeading(new Vector2d(61, 55 * flipConstant), pi/2 * flipConstant)
//                    .splineToConstantHeading(new Vector2d(61, 60 * flipConstant), pi/2 * flipConstant, lowVelocity) // slowed
//
//                    .build();
        }

        public Action humanBallsToLaunchZone() {
            return drive.actionBuilder(new Pose2d(61, 60 * flipConstant, pi/2 * flipConstant)) /// this should be the human ball pose
                    .setTangent(3*pi/2 * flipConstant)
                    .splineToConstantHeading(new Vector2d(56, 30 * flipConstant), 3*pi/2 * flipConstant)
                    .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), 3*pi/2 * flipConstant)

                    .build();
        }

        public Action launchZoneToClassifier() {
            return drive.actionBuilder(launchPose)
                    .splineToSplineHeading(new Pose2d(12,58, 5*pi/8), 3*pi/4)

                    .build();
        }

        public Action launchZoneToSweepDrew() {
            return drive.actionBuilder(launchPose)
                    .setTangent(pi/2 * flipConstant)
                    .splineToSplineHeading(new Pose2d(59, 35 * flipConstant, pi/2 * flipConstant), pi/2 * flipConstant, mediumVelocity)
                    .splineToSplineHeading(new Pose2d(40, 60 * flipConstant, pi), pi, mediumVelocity)
                    .splineToSplineHeading(new Pose2d(20, 60 * flipConstant, pi), pi, mediumVelocity)

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

        public Action launchZoneToSweepTwoPart() {
            return drive.actionBuilder(launchPose)
                    // first part of sweep
                    .setTangent(pi/2 * flipConstant)
                    .splineToSplineHeading(new Pose2d(56, 30 * flipConstant, pi/2 * flipConstant), pi/2 * flipConstant)
                    .splineToConstantHeading(new Vector2d(61, 45 * flipConstant), pi/2 * flipConstant)
                    .splineToConstantHeading(new Vector2d(61, 60 * flipConstant), pi/2 * flipConstant)

                    // going back
                    .setTangent(3*pi/2 * flipConstant)
                    .splineToConstantHeading(new Vector2d(60, 40 * flipConstant), 3*pi/2 * flipConstant)

                    // second part of sweep
                    .setTangent(pi/2 * flipConstant)
                    .splineToSplineHeading(new Pose2d(40, 60 * flipConstant, pi), pi)
                    .splineToSplineHeading(new Pose2d(30, 60 * flipConstant, pi), pi)

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

            vision = new AutoVision(hardwareMap, useFarAuto());
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
                telemetry.update();
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

                    for (int i = 0; i < 3; i++) {
                        if (spindex.ballStates[i] == Spindex.BallState.EMPTY) {
                            spindex.ballStates[i] = Spindex.BallState.PURPLE;
                        }
                    }

                    spindex.intakeMotor.setPower(0.5);

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
            private ElapsedTime spitTimer;
            private double spitTime = 0.5;
            private boolean hasSpit = false;

            public ReadyOuttake(double launchDistance, boolean sortBalls) {
                this.launchDistance = launchDistance;
                this.sortBalls = sortBalls;
                this.spitTimer = new ElapsedTime(ElapsedTime.SECOND_IN_NANO);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    // TODO set all empty drum positions to purple IF we're keep that part in the LaunchAllBalls class

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

                if (!hasSpit && !spindex.drumIsSwitching()) {
                    spitTimer.reset();
                    hasSpit = true;
                }

                if (spitTimer.seconds() < spitTime) {
                    spindex.intakeMotor.setPower(-1);
                } else if (hasSpit) {
                    spindex.intakeMotor.setPower(0.5);
                } else {
                    spindex.intakeMotor.setPower(1);
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

        public class DrumIsEmpty implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return spindex.drumIsEmpty();
            }
        }

        /// keeps running as long as the drum is empty
        public Action drumIsEmpty() {
            return new DrumIsEmpty();
        }
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }

    public final double pi = Math.PI;

    Pose2d beginPose;
    MecanumDrive drive;

    VelConstraint lowVelocity;
    VelConstraint mediumVelocity;
    BallHandler ballHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        //Vision vision = new Vision(hardwareMap, allianceIsRed());
        ballHandler = new BallHandler(hardwareMap);

        lowVelocity = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 15;
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
        FarPathGenerator path = new FarPathGenerator();

        /// -------------------------------- PRELOAD --------------------------------
        Action preload = new ParallelAction(path.startToLaunchZone(), ballHandler.launchAllBalls(3.2, true));


        /// -------------------------------- HUMAN BALLS --------------------------------
        Action humanBalls = new SequentialAction(
                // get human balls
                new RaceAction(
                        new SequentialAction(path.launchZoneToHumanBalls(), new SleepAction(0.5)),
                        ballHandler.runActiveIntake(true)
                ),

                // go back to launch zone
                new RaceAction(path.humanBallsToLaunchZone(), ballHandler.readyOuttake(3.2, true)),

                // fire human balls
                new RaceAction(ballHandler.launchAllBalls(3.2, true), path.updateLocalizer())
        );


        /// -------------------------------- FAR BALLS --------------------------------
        Action farBalls = new SequentialAction(
                // get far balls
                new RaceAction(path.launchZoneToFarBalls(), ballHandler.runActiveIntake(true)),

                // go back to launch zone
                new RaceAction(path.toLaunchZone(), ballHandler.readyOuttake(3.2, true)),

                // fire far balls
                new RaceAction(ballHandler.launchAllBalls(3.2, true), path.updateLocalizer())
        );


        /// -------------------------------- MIDDLE BALLS --------------------------------
        Action middleBalls = new SequentialAction(
                // get middle balls
                new RaceAction(path.launchZoneToMiddleBalls(), ballHandler.runActiveIntake(true)),

                // go back to launch zone
                new RaceAction(path.toLaunchZone(), ballHandler.readyOuttake(3.2, true)),

                // fire middle balls
                new RaceAction(ballHandler.launchAllBalls(3.2, true), path.updateLocalizer())
        );


        /// -------------------------------- PARK --------------------------------
        Action park = path.launchZoneToPark();


        /// -------------------------------- SWEEP --------------------------------
        Action sweep = new SequentialAction(
                // sweep while running intake
                new RaceAction(path.launchZoneToSweepTwoPart(), ballHandler.runActiveIntake(true)),

                // robot waits until it has at least 1 ball
                new RaceAction(ballHandler.runActiveIntake(false), ballHandler.drumIsEmpty(), path.updateLocalizer()),

                // go back to launch zone
                new RaceAction(path.toLaunchZoneStraight(), ballHandler.readyOuttake(3.2, true)),

                // launch swept balls
                new RaceAction(ballHandler.launchAllBalls(3.2, true), path.updateLocalizer())
        );

        /// IMPORTANT!! main auto plan:
        Action autonomous = new SequentialAction(
                preload,
                humanBalls,
                farBalls,
                middleBalls
        );

        /// once 28.5 seconds have passed, stop everything and park
        autonomous = new SequentialAction(
                new RaceAction(autonomous, new SleepAction(28.5)),
                park
        );

        autonomous = new RaceAction(
                autonomous,
                recordPose()
        );

        ballHandler.init();

        while (opModeInInit() && !isStopRequested()) {
            ballHandler.vision.detectObelisk();
            telemetry.addData("obelisk", ballHandler.vision.obeliskId);
            telemetry.addLine("21 is gpp; 22 is pgp; 23 is ppg");
            telemetry.update();
        }

        Actions.runBlocking(autonomous);
    }

    public void runCloseAutoBlocking() {
        ClosePathGenerator path = new ClosePathGenerator();

        Action autonomous = new SequentialAction(

                /// -------------------------------- SHOOT PRELOAD + MIDDLE BALLS --------------------------------

                // shoot preload while moving, then intake middle balls
                new RaceAction(
                        new SequentialAction(ballHandler.launchAllBalls(0, false), ballHandler.runActiveIntake(true)),
                        path.startToMiddleBalls()
                ),

                // go to launch zone
                new RaceAction(ballHandler.readyOuttake(1.5, false), path.toLaunchZone()),

                // shoot middle balls
                new RaceAction(ballHandler.launchAllBalls(1.5, false), path.updateLocalizer()),

                /// -------------------------------- CLEAR CLASSIFIER --------------------------------

                // go to classifier and intake (max 8 seconds)
                new RaceAction(
                        ballHandler.runActiveIntake(true),
                        new SequentialAction(path.launchZoneToClassifier(), path.updateLocalizer()),
                        new SleepAction(8)
                ),

                // go to launch zone
                new RaceAction(ballHandler.readyOuttake(1.5), path.classifierToLaunchZone()),

                // shoot classifier balls
                new RaceAction(ballHandler.launchAllBalls(1.5), path.updateLocalizer()),

                /// -------------------------------- CLOSE BALLS --------------------------------

                // intake close balls
                new RaceAction(ballHandler.runActiveIntake(true), path.launchZoneToCloseBalls()),

                // go to final launch zone
                new RaceAction(ballHandler.readyOuttake(0.7), path.toFinalLaunchZone()),

                // launch close balls
                ballHandler.launchAllBalls(0.7)
        );

        autonomous = new RaceAction(
                autonomous,
                ballHandler.trackObelisk(true),
                recordPose()
        );

        waitForStart();

        ballHandler.init();

        Actions.runBlocking(autonomous);
    }
}

class AutoVision {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public int obeliskId = 0;
    public Servo cameraServo;
    private final Vector2d obeliskPosition = new Vector2d(-72, 0);

    public AutoVision(HardwareMap hardwareMap, boolean useHighQuality) {
        // pretty sure that orientation doesn't matter for this
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        if (useHighQuality) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                    .setCameraResolution(new Size(1920, 1080))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                    .addProcessor(aprilTag)
                    .build();
        }

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