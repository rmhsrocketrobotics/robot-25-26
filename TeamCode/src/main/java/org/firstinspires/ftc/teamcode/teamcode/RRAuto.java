package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public final class RRAuto extends LinearOpMode {
    public final double pi = Math.PI;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(38, 62, pi/2);
        Pose2d bucketPose = new Pose2d(54, 58, 3*pi/4);
        Pose2d leftBlockPose = new Pose2d(48, 44, 3*pi/2);
        Pose2d middleBlockPose = new Pose2d(59, 44, 3*pi/2);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // TODO type stuff: actually made this work
                        .build());
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }
}