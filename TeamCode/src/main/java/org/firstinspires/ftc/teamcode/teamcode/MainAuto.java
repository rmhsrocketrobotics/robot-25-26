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
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.State;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Velocity;

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
        public int obeliskId;

        public BallHandler(HardwareMap hardwareMap) {
            spindex = new Spindex(hardwareMap);
            spindex.ballStates = new Spindex.BallState[]{Spindex.BallState.GREEN, Spindex.BallState.PURPLE, Spindex.BallState.PURPLE};

            outtake = new Outtake(hardwareMap);
            outtake.tolerance = 100;
        }

        public void init() {
            spindex.init();
            outtake.init();
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

                    if (obeliskId == 21) {
                        spindex.queueBall("green");
                        spindex.queueBall("purple");
                        spindex.queueBall("purple");

                    } else if (obeliskId == 22) {
                        spindex.queueBall("purple");
                        spindex.queueBall("green");
                        spindex.queueBall("purple");

                    } else if (obeliskId == 23) {
                        spindex.queueBall("purple");
                        spindex.queueBall("purple");
                        spindex.queueBall("green");
                    }

//                    outtake.setOuttakeToSpeed(launchVelocity.speed, 3.5); // TODO fix this once the new shooting system is done
//                    outtake.setHoodServoToAngle(launchVelocity.direction);
                }

                spindex.update(outtake, State.OUTTAKE);
                outtake.update();

                outtake.printTelemetry(telemetry);
                spindex.printTelemetry(telemetry);

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

            public ReadyOuttake(double launchDistance) {
                this.launchDistance = launchDistance;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    spindex.intakeMotor.setPower(0);

//                    outtake.setOuttakeToSpeed(launchVelocity.speed, 3.5); // TODO see above todo
//                    outtake.setHoodServoToAngle(launchVelocity.direction);

                }

                spindex.update(outtake, State.OUTTAKE);
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
        if (useFarAuto()) {
            launchDistance = 3; // TODO: actually get good values for this
        } else {
            launchDistance = 1.5;
        }


        // path actions combined with other actions
        Action launchFirstBalls = new ParallelAction(startToLaunchZone.build(), ballHandler.launchAllBalls(launchDistance));

        Action getSecondBalls = new RaceAction(launchZoneToSecondBalls.build(), ballHandler.runActiveIntake(true));
//        Action returnToLaunchZoneWithSecondBalls = new RaceAction(secondBallsToLaunchZone.build(), ballHandler.readyOuttake(launchVelocity));
        Action returnToLaunchZoneWithSecondBalls = new RaceAction(secondBallsToLaunchZone.build(), ballHandler.runActiveIntake(false));
        Action launchSecondBalls = ballHandler.launchAllBalls(launchDistance);

        Action getThirdBalls = new RaceAction(launchZoneToThirdBalls.build(), ballHandler.runActiveIntake(true));
//        Action returnToLaunchZoneWithThirdBalls = new RaceAction(thirdBallsToLaunchZone.build(), ballHandler.readyOuttake(launchVelocity));
        Action returnToLaunchZoneWithThirdBalls = new RaceAction(thirdBallsToLaunchZone.build(), ballHandler.runActiveIntake(false));
        Action launchThirdBalls = ballHandler.launchAllBalls(launchDistance);

        // combine all of the above actions into one big long sequential action
        Action runAutonomous = new SequentialAction(
                launchFirstBalls,
                getSecondBalls, returnToLaunchZoneWithSecondBalls, launchSecondBalls,
                getThirdBalls, returnToLaunchZoneWithThirdBalls, launchThirdBalls
        );

        // add pose recording to the auto action
        runAutonomous = new RaceAction(runAutonomous, recordPose());

        // this is in place of a waitForStart() call
        waitForStart();
//        while (opModeInInit() && !isStopRequested()) {
//            vision.detectObelisk();
//            telemetry.addData("obelisk", vision.obeliskId);
//            telemetry.addLine("21 is gpp; 22 is pgp; 23 is ppg");
//        }

        ballHandler.obeliskId = 22;//vision.obeliskId;
        ballHandler.init();

        Actions.runBlocking(runAutonomous);
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }

    public void farPath() {
        beginPose = new Pose2d(61, 13.5, Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, beginPose);

        Vector2d launchPosition = new Vector2d(56, 14.5);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-66, 58));

        TrajectoryActionBuilder actionBuilder;
        if (allianceIsRed()) {
            actionBuilder = drive.actionBuilder(beginPose);
        } else {
            actionBuilder = drive.actionBuilder(beginPose, pose2dDual -> new Pose2dDual<>(
                    pose2dDual.position.x, pose2dDual.position.y.unaryMinus(), pose2dDual.heading.plus(Math.PI)));
        }

        startToLaunchZone = actionBuilder
                .splineTo(launchPosition, launchToGoalAngle);

        launchZoneToSecondBalls = startToLaunchZone.endTrajectory().fresh()
                .splineTo(new Vector2d(36, 28), pi/2)
                .splineTo(new Vector2d(36, 45), pi/2, lowVelocity); // slow mode

        secondBallsToLaunchZone = launchZoneToSecondBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(launchPosition, launchToGoalAngle - pi);

        launchZoneToThirdBalls = secondBallsToLaunchZone.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(13, 28), pi/2)
                .splineTo(new Vector2d(13, 45), pi/2, lowVelocity); // slow mode

        thirdBallsToLaunchZone = launchZoneToThirdBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(launchPosition, launchToGoalAngle - pi);
    }

    public void closePath() {
        int ballPickupYPos = 27;

        beginPose = new Pose2d(-61, 36, Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, beginPose);

        Vector2d launchPosition = new Vector2d(-25, ballPickupYPos);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-58, 58));

        Vector2d launchPositionFinal = new Vector2d(-50, ballPickupYPos);
        double launchToGoalAngleFinal = angleBetweenPoints(launchPositionFinal, new Vector2d(-58, 58));

        Vector2d ball1PickupPosition = new Vector2d(-11, 55);
        double ball1ToLaunchAngle = angleBetweenPoints(ball1PickupPosition, launchPosition);

        Vector2d ball2PickupPosition = new Vector2d(13, 61);
        double ball2ToLaunchAngle = angleBetweenPoints(ball2PickupPosition, launchPosition);

        TrajectoryActionBuilder actionBuilder;
        if (allianceIsRed()) {
            actionBuilder = drive.actionBuilder(beginPose);
        } else {
            actionBuilder = drive.actionBuilder(beginPose, pose2dDual -> new Pose2dDual<>(
                    pose2dDual.position.x, pose2dDual.position.y.unaryMinus(), pose2dDual.heading.plus(Math.PI)));
        }

        startToLaunchZone = actionBuilder
                .setReversed(true)
                .splineTo(launchPosition, launchToGoalAngle - pi);

        launchZoneToSecondBalls = startToLaunchZone.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-11, ballPickupYPos, pi/2), 0)
                .setTangent(pi/2)
                .splineTo(ball1PickupPosition, pi/2, lowVelocity); // slow mode

        secondBallsToLaunchZone = launchZoneToSecondBalls.endTrajectory().fresh()
                .setTangent(ball1ToLaunchAngle)
                .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), ball1ToLaunchAngle);

        launchZoneToThirdBalls = secondBallsToLaunchZone.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(13, ballPickupYPos, pi/2), 0)
                .setTangent(pi/2)
                .splineTo(ball2PickupPosition, pi/2, lowVelocity); // slow mode

        thirdBallsToLaunchZone = launchZoneToThirdBalls.endTrajectory().fresh()
                .setTangent(3*pi/2)
                .splineToSplineHeading(new Pose2d(launchPositionFinal, launchToGoalAngleFinal), pi);

    }
}