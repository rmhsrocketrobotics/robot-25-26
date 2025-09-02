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

        double bucketToLeftAngle = angleBetweenPoints(bucketPose.position, leftBlockPose.position);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // going to bucket
                        .setTangent(-pi/4)
                        .splineToLinearHeading(bucketPose, 0)
                        .waitSeconds(1) //depositing sample

                        // going to left sample
                        .setTangent(bucketToLeftAngle)
                        .splineToSplineHeading(leftBlockPose, bucketToLeftAngle)
                        .waitSeconds(1) //picking up sample

                        // going to bucket
                        .setTangent(bucketToLeftAngle + pi)
                        .splineToSplineHeading(bucketPose, bucketToLeftAngle + pi)
                        .waitSeconds(1) //depositing sample

                        // going to middle sample
                        .setTangent(3*pi/2)
                        .splineToSplineHeading(middleBlockPose, -pi/4)
                        .waitSeconds(1) //picking up sample

                        // going to bucket
                        .setTangent(3*pi/4)
                        .splineToSplineHeading(bucketPose, pi/2)
                        .waitSeconds(1) //depositing sample

                        // push right sample
                        .setTangent(11*pi/8)
                        .splineToSplineHeading(new Pose2d(49, 8, 0), 0)
                        .splineToConstantHeading(new Vector2d(61, 45), pi/2)

                        // level 1 hang
                        .setTangent(5*pi/4)
                        .splineToConstantHeading(new Vector2d(40, 12), pi)
                        .splineToConstantHeading(new Vector2d(25, 12), pi)
                        .build());
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }
}