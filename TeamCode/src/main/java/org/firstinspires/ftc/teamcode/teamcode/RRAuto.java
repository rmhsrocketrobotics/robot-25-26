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
                        // go to the bucket
                        .waitSeconds(1)
                        .setTangent(findLineToPointAngle(beginPose, bucketPose))
                        .lineToXSplineHeading(bucketPose.position.x, bucketPose.heading)
                        .setTangent(0)
                        .waitSeconds(1) //depositing sample

                        // go to left sample
                        .splineToSplineHeading(leftBlockPose, 3*pi/2)
                        .waitSeconds(1) //picking up sample

                        // go to the bucket
                        .setTangent(findLineToPointAngle(leftBlockPose, bucketPose))
                        .lineToXSplineHeading(bucketPose.position.x, bucketPose.heading)
                        .setTangent(0)
                        .waitSeconds(1) //depositing sample

                        // go to middle sample
                        .setTangent(findLineToPointAngle(bucketPose, middleBlockPose))
                        .lineToXSplineHeading(middleBlockPose.position.x, middleBlockPose.heading)
                        .setTangent(0)
                        .waitSeconds(1) //picking up sample

                        // go to the bucket
                        .setTangent(findLineToPointAngle(middleBlockPose, bucketPose))
                        .lineToXSplineHeading(bucketPose.position.x, bucketPose.heading)
                        .setTangent(0)
                        .waitSeconds(1) //depositing sample

                        .lineToSplineHeading(new Pose2d(50, 50, 3*pi/2))
                        .splineToSplineHeading(new Pose2d(61, 10, 0), 0)
                        .lineTo(new Vector2d(61, 45))
                        .splineToConstantHeading(new Vector2d(40, 12), 0)
                        .lineTo(new Vector2d(25, 12))

                        .waitSeconds(2)
                        .build());
    }

    public double findLineToPointAngle(Pose2d currentPose, Pose2d targetPose) {
        double centeredX = targetPose.position.x - currentPose.position.x;
        double centeredY = targetPose.position.y - currentPose.position.y;

        return Math.atan(centeredY/centeredX) - currentPose.heading.toDouble();
    }
}
