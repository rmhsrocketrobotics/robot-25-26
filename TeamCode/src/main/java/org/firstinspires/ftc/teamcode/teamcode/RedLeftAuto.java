//package org.firstinspires.ftc.teamcode.teamcode;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@TeleOp
//public final class RedLeftAuto extends LinearOpMode {
//    public final double pi = Math.PI;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        double startToBall1Angle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(38, 46));
//        System.out.println(Math.toDegrees(startToBall1Angle));
//        double startToGoalAngle = angleBetweenPoints(new Vector2d(-50, 20), new Vector2d(-66, 60));
//        System.out.println(Math.toDegrees(startToGoalAngle));
//        double startToBall2Angle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(13, 43));
//        System.out.println(Math.toDegrees(startToGoalAngle));
//        double startToBallLine = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(46, 37));
//        System.out.println(Math.toDegrees(startToGoalAngle));
//
//        Pose2d beginPose = new Pose2d(-61, 20, Math.toRadians(180));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//        waitForStart();
//
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//
//                        .setReversed(true).lineToSplineHeading(new Pose2d(-50, 20, startToGoalAngle))
//
//                        .waitSeconds(3)
//
//                        .setReversed(false).lineToSplineHeading(new Pose2d(-11, 20, pi/2))
//                        .forward(25)
//
//                        .waitSeconds(3)
//
//                        .setReversed(true).lineToSplineHeading(new Pose2d(-25, 27, startToGoalAngle + pi/8))
//
//                        .waitSeconds(3)
//
//                        .setReversed(false).lineToSplineHeading(new Pose2d(13, 27, pi/2))
//                        .forward(25)
//
//                        .waitSeconds(3)
//
//                        .setReversed(true).lineToSplineHeading(new Pose2d(-25, 27, startToGoalAngle+pi/8))
//
//                        .waitSeconds(3)
//
//                        .strafeLeft(15)
//
//                        .build());
//    }
//
//    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
//        double x = point2.x - point1.x;
//        double y = point2.y - point1.y;
//
//        return Math.atan2(y, x);
//    }
//}