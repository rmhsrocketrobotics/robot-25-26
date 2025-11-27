// gameing: https://docs.google.com/document/d/1HVnt0-H7hon_oCMTfqz0AD4EIamUn2VZ2qhZOWYZUYw/edit?tab=t.0


// code copied according to https://github.com/rh-robotics/MeepMeep/blob/master/INSTALL.md


package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;


import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.SampleMecanumDrive;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


import java.awt.Image;
import java.io.File;
import java.io.IOException;


import javax.imageio.ImageIO;

public class MeepMeepTesting {
    static final double pi = Math.PI;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = testPathLeftRed(meepMeep);

        Image img = null;
        try { img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/decode webfield.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static RoadRunnerBotEntity testPath(MeepMeep meepMeep) {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .splineTo(new Vector2d(40, 40), pi/2)
                        .build());
    }


    //starts on the right side, towards red goal
    public static RoadRunnerBotEntity testPathRightRed(MeepMeep meepMeepRightRed) {
        double startToBall1Angle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(38, 46));
        System.out.println(Math.toDegrees(startToBall1Angle));
        double startToGoalAngle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(-66, 60));
        System.out.println(Math.toDegrees(startToGoalAngle));
        double startToBall2Angle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(13, 43));
        System.out.println(Math.toDegrees(startToGoalAngle));
        double startToBallLine = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(46, 37));
        System.out.println(Math.toDegrees(startToGoalAngle));


        return new DefaultBotBuilder(meepMeepRightRed)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, 10, Math.toRadians(180)))
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
                        .forward(15)

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


    //starts on the left side, aims towards red goal
    public static RoadRunnerBotEntity testPathLeftRed(MeepMeep meepMeepLeftRed) {
        double startToBall1Angle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(38, 46));
        System.out.println(Math.toDegrees(startToBall1Angle));
        double startToGoalAngle = angleBetweenPoints(new Vector2d(-50, 20), new Vector2d(-66, 60));
        System.out.println(Math.toDegrees(startToGoalAngle));
        double startToBall2Angle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(13, 43));
        System.out.println(Math.toDegrees(startToGoalAngle));
        double startToBallLine = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(46, 37));
        System.out.println(Math.toDegrees(startToGoalAngle));

        return new DefaultBotBuilder(meepMeepLeftRed)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, 20, Math.toRadians(180)))


                        .setReversed(true).splineToLinearHeading(new Pose2d(-25, 27, startToGoalAngle + pi/8), pi/8)

                        .waitSeconds(3)

                        .setTangent(0).splineToLinearHeading(new Pose2d(-11, 30, pi/2), pi/8)
                        .forward(25)

                        .waitSeconds(3)

                        .setReversed(true).splineToLinearHeading(new Pose2d(-25, 30, startToGoalAngle + pi/8), pi)

                        .waitSeconds(3)

                        .setReversed(true).splineToLinearHeading(new Pose2d(13, 30, pi/2), pi/8)
                        .forward(25)

                        .waitSeconds(3)

                        .setReversed(true).splineToLinearHeading(new Pose2d(-25, 27, startToGoalAngle + pi/8), pi)

                        .waitSeconds(3)
// third column of artifacts
//                        .setReversed(false).splineToLinearHeading(new Pose2d(36, 30, pi/2), pi/8)
//                        .forward(25)
//
//                        .waitSeconds(1.5)
//
//                        .setReversed(true).splineToLinearHeading(new Pose2d(-25, 27, startToGoalAngle+pi/8), pi)
//
//                        .waitSeconds(1.5)

                        .build());
    }
    //starts on the right side, aims towards blue goal
    public static RoadRunnerBotEntity testPathRightBlue(MeepMeep meepMeepRightBlue) {
        double startToBallOneAngle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(38, -46));
        System.out.println(Math.toDegrees(startToBallOneAngle));
        double startToGoalAngle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(-66, -60));
        System.out.println(Math.toDegrees(startToGoalAngle));
        double startToBallTwoAngle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(13, -43));
        System.out.println(Math.toDegrees(startToGoalAngle));
        double startToBallLine = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(46, -37));
        System.out.println(Math.toDegrees(startToGoalAngle));

        return new DefaultBotBuilder(meepMeepRightBlue)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, -10, Math.toRadians(180)))
                        .splineTo(new Vector2d(51, -13),startToGoalAngle)

                        .waitSeconds(3)

                        .splineTo(new Vector2d(36, -38),3*pi/2)
                        .splineTo(new Vector2d(36, -46), 3*pi/2)

                        .waitSeconds(3)

                        .setReversed(true).splineTo(new Vector2d(56, -10), startToGoalAngle - pi)

                        .waitSeconds(3)

                        .setReversed(false).splineTo(new Vector2d(13, -37), 3*pi/2)
                        .splineTo(new Vector2d(13, -46), 3*pi/2)

                        .waitSeconds(3)

                        .setReversed(true).splineTo(new Vector2d(56, -10), startToGoalAngle+pi)
                        .waitSeconds(3)
                        .forward(15)
// third column of artifacts
//                        .setReversed(false).splineTo(new Vector2d(-11, -37), 3*pi/2)
//                        .splineTo(new Vector2d(-11, -46), 3*pi/2)
//
//                        .waitSeconds(3)
//
//                        .setReversed(true).splineTo(new Vector2d(56, -10), startToGoalAngle+pi)
//                        .waitSeconds(3)

                        .build());
    }

    //starts on the left side, aims towards blue goal
    public static RoadRunnerBotEntity testPathLeftBlue(MeepMeep meepMeepLeftBlue) {
        double startToBall1Angle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(38, 46));
        System.out.println(Math.toDegrees(startToBall1Angle));
        double startToGoalAngle = angleBetweenPoints(new Vector2d(-50, 20), new Vector2d(-66, 60));
        System.out.println(Math.toDegrees(startToGoalAngle));
        double startToBall2Angle = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(13, 43));
        System.out.println(Math.toDegrees(startToGoalAngle));
        double startToBallLine = angleBetweenPoints(new Vector2d(56, 0), new Vector2d(46, 37));
        System.out.println(Math.toDegrees(startToGoalAngle));

        return new DefaultBotBuilder(meepMeepLeftBlue)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, -20, Math.toRadians(180)))

                        .setReversed(true).splineToLinearHeading(new Pose2d(-25, -20, -startToGoalAngle - pi/8), -pi/8)

                        .waitSeconds(3)
//
                        .setReversed(false).splineToLinearHeading(new Pose2d(-11, -15, 3*pi/2), -pi/8)
                        .forward(25)

                        .waitSeconds(3)

                        .setReversed(true).splineToLinearHeading(new Pose2d(-25, -27, -startToGoalAngle - pi/8), -pi)

                        .waitSeconds(3)

                        .setReversed(false).splineToLinearHeading(new Pose2d(13, -20, 3*pi/2), -pi/8)
                        .forward(25)

                        .waitSeconds(3)

                        .setReversed(true).splineToLinearHeading(new Pose2d(-25, -27, -startToGoalAngle - pi/8), -pi)

                        .waitSeconds(3)
                        .strafeLeft(15)
////// third column of artifacts
//                        .setReversed(false).splineToLinearHeading(new Pose2d(36, -27, 3*pi/2), -pi/8)
//                        .forward(25)

////                        .waitSeconds(1.5)

//                        .setReversed(true).splineToLinearHeading(new Pose2d(-25, -27, -startToGoalAngle-pi/8), -pi)

//                        .waitSeconds(1.5)

                        .build());
    }

    static double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.getX() - point1.getX();
        double y = point2.getY() - point1.getY();

        return Math.atan2(y, x);
    }
}


