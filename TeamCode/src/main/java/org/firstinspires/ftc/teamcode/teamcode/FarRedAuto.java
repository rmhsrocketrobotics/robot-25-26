package org.firstinspires.ftc.teamcode.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Velocity;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Vision;

@Autonomous
public final class FarRedAuto extends LinearOpMode {

    public class BallHandler { // combination of spindex and outtake
        Spindex spindex;
        Outtake outtake;
        public int obeliskId;

        public BallHandler(HardwareMap hardwareMap) {
            spindex = new Spindex(hardwareMap, true);
            spindex.ballStates = new String[]{"green", "purple", "purple"};

            outtake = new Outtake(hardwareMap);
            outtake.tolerance = 150;
        }

        public void init() {
            spindex.init();
        }

        public class LaunchAllBalls implements Action {
            private boolean initialized = false;

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
                    outtake.setOuttakeToSpeed(6.25);
                    outtake.setHoodServoToAngle(45);
                }

                spindex.update(outtake);
                outtake.update();

                return !spindex.shouldSwitchToIntake;
            }
        }
        public Action launchAllBalls() {
            return new LaunchAllBalls();
        }

        public class RunActiveIntake implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    spindex.ballStates = new String[] {};
                    spindex.setDrumState("intake", 0);
                    spindex.intake.setPower(1);
                }
                spindex.update(outtake);

                return true;
            }
        }
        public Action runActiveIntake() {
            return new RunActiveIntake();
        }

        public class ReadyOuttake implements Action {
            private boolean initialized = false;
            public Velocity ballVelocity;

            public ReadyOuttake(Velocity ballVelocity) {
                this.ballVelocity = ballVelocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    spindex.setDrumState("outtake", 0);
                    spindex.update(outtake);

                    outtake.setOuttakeToSpeed(6.25);
                    outtake.setHoodServoToAngle(45);
                }

                return true;
            }
        }
        public Action readyOuttake(Velocity ballVelocity) {
            return new ReadyOuttake(ballVelocity);
        }
    }

    public final double pi = Math.PI;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(61, 10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Vision vision = new Vision(hardwareMap, true);
        BallHandler ballHandler = new BallHandler(hardwareMap);

        double startToGoalAngle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(-66, 60));

        // path actions: (.build() must be called in order for any of these to become actual actions)
        // IMPORTANT: for all of the following functions:
        // "first balls" are the PRELOAD;
        // "second balls" are the FIRST set of balls we PICK UP;
        // "third balls" are the SECOND set of balls we PICK UP
        TrajectoryActionBuilder startToLaunchZone = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(51, 13), startToGoalAngle);

        TrajectoryActionBuilder launchZoneToSecondBalls = startToLaunchZone.endTrajectory().fresh()
                .splineTo(new Vector2d(36, 38), pi/2)
                .splineTo(new Vector2d(36, 46), pi/2);

        TrajectoryActionBuilder secondBallsToLaunchZone = launchZoneToSecondBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(56, 10), startToGoalAngle + pi);

        TrajectoryActionBuilder launchZoneToThirdBalls = secondBallsToLaunchZone.endTrajectory().fresh()
                .setReversed(false).splineTo(new Vector2d(13, 37), pi/2)
                .splineTo(new Vector2d(13, 46), pi/2);

        TrajectoryActionBuilder thirdBallsToLaunchZone = launchZoneToThirdBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(56, 10), startToGoalAngle+pi);

        Velocity launchVelocity = new Velocity(6.25, 45);

        // path actions combined with other actions
        Action launchFirstBalls = new ParallelAction(startToLaunchZone.build(), ballHandler.launchAllBalls());

        Action getSecondBalls = new RaceAction(launchZoneToSecondBalls.build(), ballHandler.runActiveIntake());
        Action returnToLaunchZoneWithSecondBalls = new RaceAction(secondBallsToLaunchZone.build(), ballHandler.readyOuttake(launchVelocity));
        Action launchSecondBalls = ballHandler.launchAllBalls();

        Action getThirdBalls = new RaceAction(launchZoneToThirdBalls.build(), ballHandler.runActiveIntake());
        Action returnToLaunchZoneWithThirdBalls = new RaceAction(thirdBallsToLaunchZone.build(), ballHandler.readyOuttake(launchVelocity));
        Action launchThirdBalls = ballHandler.launchAllBalls();

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

        Actions.runBlocking(runAutonomous);

//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//
//                        .splineTo(new Vector2d(51, 13),startToGoalAngle)
//                        .waitSeconds(3)
//
//                        .splineTo(new Vector2d(36, 38),pi/2)
//                        .splineTo(new Vector2d(36, 46), pi/2)
//
//                        .waitSeconds(3)
//
//                        .setReversed(true).splineTo(new Vector2d(56, 10), startToGoalAngle + pi)
//
//                        .waitSeconds(3)
//
//                        .setReversed(false).splineTo(new Vector2d(13, 37), pi/2)
//                        .splineTo(new Vector2d(13, 46), pi/2)
//
//                        .waitSeconds(3)
//
//                        .setReversed(true).splineTo(new Vector2d(56, 10), startToGoalAngle+pi)
//                        .waitSeconds(3)
//
//                        .build());
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }
}