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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Velocity;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Vision;

@Autonomous(preselectTeleOp = "MainTeleop")
public final class MainAuto extends LinearOpMode {
    public boolean isRedAlliance = false;
    public boolean isFar = false;

    public class BallHandler { // combination of spindex and outtake
        Spindex spindex;
        Outtake outtake;
        public int obeliskId;

        public BallHandler(HardwareMap hardwareMap) {
            spindex = new Spindex(hardwareMap, true);
            spindex.ballStates = new String[]{"green", "purple", "purple"};

            outtake = new Outtake(hardwareMap);
            outtake.tolerance = 100;
        }

        public void init() {
            spindex.init();
        }

        public class LaunchAllBalls implements Action {
            private boolean initialized = false;
            Velocity launchVelocity;

            public LaunchAllBalls(Velocity launchVelocity) {
                this.launchVelocity = launchVelocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    spindex.setDrumState("outtake", 0);

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
                    outtake.setOuttakeToSpeed(launchVelocity.speed, 3.5);
                    outtake.setHoodServoToAngle(launchVelocity.direction);
                }

                spindex.update(outtake);
                outtake.update(spindex);

                outtake.printTelemetry(telemetry);
                spindex.printTelemetry(telemetry);

                telemetry.update();

                return !spindex.shouldSwitchToIntake;
            }
        }
        public Action launchAllBalls(Velocity launchVelocity) {
            return new LaunchAllBalls(launchVelocity);
        }

        public class RunActiveIntake implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    outtake.targetTicksPerSecond = 0;

                    spindex.ballStates = new String[] {"empty", "empty", "empty"};
                    spindex.setDrumState("intake", 0);
                    spindex.intake.setPower(1);
                }
                spindex.update(outtake);
                outtake.update(spindex);

                return true;
            }
        }
        public Action runActiveIntake() {
            return new RunActiveIntake();
        }

        public class ReadyOuttake implements Action {
            private boolean initialized = false;
            public Velocity launchVelocity;

            public ReadyOuttake(Velocity launchVelocity) {
                this.launchVelocity = launchVelocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    spindex.intake.setPower(0);
                    spindex.setDrumState("outtake", 0);

                    outtake.setOuttakeToSpeed(launchVelocity.speed, 3.5);
                    outtake.setHoodServoToAngle(launchVelocity.direction);

                }

                spindex.update(outtake);
                outtake.update(spindex);

                return true;
            }
        }
        public Action readyOuttake(Velocity launchVelocity) {
            return new ReadyOuttake(launchVelocity);
        }
    }

    public final double pi = Math.PI;

    VelConstraint lowVelocity;
    TrajectoryActionBuilder startToLaunchZone;
    TrajectoryActionBuilder launchZoneToSecondBalls;
    TrajectoryActionBuilder secondBallsToLaunchZone;
    TrajectoryActionBuilder launchZoneToThirdBalls;
    TrajectoryActionBuilder thirdBallsToLaunchZone;

    @Override
    public void runOpMode() throws InterruptedException {
        Vision vision = new Vision(hardwareMap, true);
        BallHandler ballHandler = new BallHandler(hardwareMap);

        ballHandler.spindex.flickTime = 1;
        ballHandler.spindex.postFlickTime = 0.4;

        lowVelocity = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 6;
            }
        };

        // path actions: (.build() must be called in order for any of these to become actual actions)
        // IMPORTANT: for all of the following functions:
        // "first balls" are the PRELOAD;
        // "second balls" are the FIRST set of balls we PICK UP;
        // "third balls" are the SECOND set of balls we PICK UP
        // just moved all of the path building into functions but this still applies

        if (isRedAlliance && isFar) {
            redFarPath();
        } else if (!isRedAlliance && isFar) {
            blueFarPath();
        } else if (isRedAlliance && !isFar) {
            redClosePath();
        } else if (!isRedAlliance && !isFar) {
            blueClosePath();
        }

        Velocity launchVelocity;
        if (isFar) {
            launchVelocity = new Velocity(6.3, 45);
        } else {
            launchVelocity = new Velocity(4.3, 55);
        }


        // path actions combined with other actions
        Action launchFirstBalls = new ParallelAction(startToLaunchZone.build(), ballHandler.launchAllBalls(launchVelocity));

        Action getSecondBalls = new RaceAction(launchZoneToSecondBalls.build(), ballHandler.runActiveIntake());
//        Action returnToLaunchZoneWithSecondBalls = new RaceAction(secondBallsToLaunchZone.build(), ballHandler.readyOuttake(launchVelocity));
        Action returnToLaunchZoneWithSecondBalls = new RaceAction(secondBallsToLaunchZone.build(), ballHandler.runActiveIntake());
        Action launchSecondBalls = ballHandler.launchAllBalls(launchVelocity);

        Action getThirdBalls = new RaceAction(launchZoneToThirdBalls.build(), ballHandler.runActiveIntake());
//        Action returnToLaunchZoneWithThirdBalls = new RaceAction(thirdBallsToLaunchZone.build(), ballHandler.readyOuttake(launchVelocity));
        Action returnToLaunchZoneWithThirdBalls = new RaceAction(thirdBallsToLaunchZone.build(), ballHandler.runActiveIntake());
        Action launchThirdBalls = ballHandler.launchAllBalls(launchVelocity);

        // combine all of the above actions into one big long sequential action
        Action runAutonomous = new SequentialAction(
                launchFirstBalls,
                getSecondBalls, returnToLaunchZoneWithSecondBalls, launchSecondBalls,
                getThirdBalls, returnToLaunchZoneWithThirdBalls, launchThirdBalls
        );

        // this is in place of a waitForStart() call
        while (opModeInInit() && !isStopRequested()) {
            vision.detectObelisk();
            telemetry.addData("obelisk", vision.obeliskId);
            telemetry.addLine("21 is gpp; 22 is pgp; 23 is ppg");
        }

        ballHandler.obeliskId = vision.obeliskId;
        ballHandler.init();

        Actions.runBlocking(runAutonomous);
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }

    public void redFarPath() {
        Pose2d beginPose = new Pose2d(61, 13.5, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Vector2d launchPosition = new Vector2d(56, 14.5);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-66, 58));

        startToLaunchZone = drive.actionBuilder(beginPose)
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

    public void blueFarPath() {
        Pose2d beginPose = new Pose2d(61, -13.5, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Vector2d launchPosition = new Vector2d(54, -15);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-66, -58));

        startToLaunchZone = drive.actionBuilder(beginPose)
                .splineTo(launchPosition, launchToGoalAngle);

        launchZoneToSecondBalls = startToLaunchZone.endTrajectory().fresh()
                .splineTo(new Vector2d(36, -28), 3*pi/2)
                .splineTo(new Vector2d(36, -55), 3*pi/2, lowVelocity); // slow mode

        secondBallsToLaunchZone = launchZoneToSecondBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(launchPosition, launchToGoalAngle - pi);

        launchZoneToThirdBalls = secondBallsToLaunchZone.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(13, -28), 3*pi/2)
                .splineTo(new Vector2d(13, -55), 3*pi/2, lowVelocity); // slow mode

        thirdBallsToLaunchZone = launchZoneToThirdBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(launchPosition, launchToGoalAngle - pi);
    }

    public void redClosePath() {
        int ballPickupYPos = 27;

        Pose2d beginPose = new Pose2d(-61, 36, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Vector2d launchPosition = new Vector2d(-25, ballPickupYPos);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-66, 58));

        Vector2d ball1PickupPosition = new Vector2d(-11, 52);
        double ball1ToLaunchAngle = angleBetweenPoints(ball1PickupPosition, launchPosition);

        Vector2d ball2PickupPosition = new Vector2d(13, 52);
        double ball2ToLaunchAngle = angleBetweenPoints(ball2PickupPosition, launchPosition);

        startToLaunchZone = drive.actionBuilder(beginPose)
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
                .setTangent(ball2ToLaunchAngle)
                .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), ball2ToLaunchAngle);

    }

    public void blueClosePath() {
        int ballPickupYPos = -27;

        Pose2d beginPose = new Pose2d(-61, -36, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Vector2d launchPosition = new Vector2d(-25, ballPickupYPos);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-60, -58));

        Vector2d ball1PickupPosition = new Vector2d(-11, -52);
        double ball1ToLaunchAngle = angleBetweenPoints(ball1PickupPosition, launchPosition);

        Vector2d ball2PickupPosition = new Vector2d(13, -52);
        double ball2ToLaunchAngle = angleBetweenPoints(ball2PickupPosition, launchPosition);

        startToLaunchZone = drive.actionBuilder(beginPose)
                .setReversed(true)
                .splineTo(launchPosition, launchToGoalAngle - pi);

        launchZoneToSecondBalls = startToLaunchZone.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-11, ballPickupYPos, 3*pi/2), 0)
                .setTangent(3*pi/2)
                .splineTo(ball1PickupPosition, 3*pi/2, lowVelocity); // slow mode

        secondBallsToLaunchZone = launchZoneToSecondBalls.endTrajectory().fresh()
                .setTangent(ball1ToLaunchAngle)
                .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), ball1ToLaunchAngle);

        launchZoneToThirdBalls = secondBallsToLaunchZone.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(13, ballPickupYPos, 3*pi/2), 0)
                .setTangent(3*pi/2)
                .splineTo(ball2PickupPosition, 3*pi/2, lowVelocity); // slow mode

        thirdBallsToLaunchZone = launchZoneToThirdBalls.endTrajectory().fresh()
                .setTangent(ball2ToLaunchAngle)
                .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), ball2ToLaunchAngle);
    }
}