package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public final class RedRightAuto extends LinearOpMode {
    public final double pi = Math.PI;

    @Override
    public void runOpMode() throws InterruptedException {
        double startToBall1Angle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(38, 46));

        double startToGoalAngle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(-66, 60));

        double startToBall2Angle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(13, 43));

        double startToBallLine = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(46, 37));

        Pose2d beginPose = new Pose2d(61, 10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

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

                        .build());
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }
}