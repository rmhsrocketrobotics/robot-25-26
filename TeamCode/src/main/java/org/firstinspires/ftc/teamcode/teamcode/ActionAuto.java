package org.firstinspires.ftc.teamcode.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Velocity;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Vision;

@TeleOp
public final class ActionAuto extends LinearOpMode {


    public class BallHandler { // combination of spindex and outtake
        Spindex spindex;
        Outtake outtake;

        public BallHandler(HardwareMap hardwareMap) {
            spindex = new Spindex(hardwareMap);
            spindex.ballStates = new String[]{"green", "purple", "purple"};

            outtake = new Outtake(hardwareMap);
            outtake.tolerance = 150;
        }

        public class LaunchAllBalls implements Action {
            private boolean initialized = false;
            int obeliskId;

            public LaunchAllBalls(int obeliskId) {
                this.obeliskId = obeliskId;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    spindex.setDrumState("outtake");
                    spindex.init();

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

//                    outtake.setOuttakeAndHoodToVelocity(new Velocity(6, 45));
                }

                spindex.update(outtake);

                return !spindex.shouldSwitchToIntake;
            }
        }
        public Action launchAllBalls(int obeliskId) {
            return new LaunchAllBalls(obeliskId);
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

        TrajectoryActionBuilder startToLaunchZone = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(51, 13),startToGoalAngle);


        TrajectoryActionBuilder launchZoneToFirstBalls = startToLaunchZone.endTrajectory().fresh()
                .splineTo(new Vector2d(36, 38), pi/2)
                .splineTo(new Vector2d(36, 46), pi/2);

        TrajectoryActionBuilder firstBallsToLaunchZone = launchZoneToFirstBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(56, 10), startToGoalAngle + pi);

        TrajectoryActionBuilder launchZoneToSecondBalls = firstBallsToLaunchZone.endTrajectory().fresh()
                .setReversed(false).splineTo(new Vector2d(13, 37), pi/2)
                .splineTo(new Vector2d(13, 46), pi/2);

        TrajectoryActionBuilder secondBallsToLaunchZone = launchZoneToSecondBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(56, 10), startToGoalAngle+pi);

        //Action shootFirstBalls = new ParallelAction(startToLaunchZone.build(), ballHandler.launchAllBalls(21));

        // this is in place of a waitForStart() call
        while (opModeInInit()) {
            vision.detectObelisk();
            telemetry.addData("obelisk", vision.obeliskId);
            telemetry.addLine("21 is gpp; 22 is pgp; 23 is ppg");
        }

        Action fullPath = new SequentialAction(startToLaunchZone.build(), launchZoneToFirstBalls.build());
        Actions.runBlocking(


                        .splineTo(new Vector2d(51, 13),startToGoalAngle)

                        .waitSeconds(3)

                        .splineTo(new Vector2d(36, 38),pi/2)
                        .splineTo(new Vector2d(36, 46), pi/2)

                        .waitSeconds(3)

                        .setReversed(true).splineTo(new Vector2d(56, 10), startToGoalAngle + pi)

                        .waitSeconds(3)

                        .setReversed(false).splineTo(new Vector2d(13, 37), pi/2)
                        .splineTo(new Vector2d(13, 46), pi/2)

                        .waitSeconds(3)

                        .setReversed(true).splineTo(new Vector2d(56, 10), startToGoalAngle+pi)
                        .waitSeconds(3)
                        .setReversed(false).splineTo(new Vector2d(40, 10), pi)

// third column of artifacts
//                        .setReversed(false).splineTo(new Vector2d(-11, 37), pi/2)
//                        .splineTo(new Vector2d(-11, 46), pi/2)
//
//                        .waitSeconds(3)
//
//                        .setReversed(true).splineTo(new Vector2d(56, 10), startToGoalAngle+pi)
//                        .waitSeconds(3)

                        .build());
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }
}
