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

        RoadRunnerBotEntity myBot = testPathFar(meepMeep);

        Image img = null;
        try { img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/decode webfield.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    //transfered competition near code
    public static RoadRunnerBotEntity testPathClose(MeepMeep meepMeepClose) {
        int ballPickupYPos = 30;

        Vector2d startLaunchPosition = new Vector2d(-23, 24);

        Vector2d launchPosition = new Vector2d(-11, 20);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-58, 58));
        Vector2d ball1PickupPosition = new Vector2d(-11, 55);
//        double ball1ToLaunchAngle = angleBetweenPoints(ball1PickupPosition, launchPosition);
        Vector2d ball2PickupPosition = new Vector2d(13, 61);

        Vector2d endLaunchPosition = new Vector2d(-46, 25);
        double endLaunchToGoalAngle = angleBetweenPoints(endLaunchPosition, new Vector2d(-58, 58));
        double ball1ToEndLaunchAngle = angleBetweenPoints(ball1PickupPosition, endLaunchPosition);


        return new DefaultBotBuilder(meepMeepClose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-48, 52, Math.toRadians(130)))
                        //startToLaunchZone
                        .setReversed(true)
                        .splineTo(startLaunchPosition, launchToGoalAngle - pi)
//                        .waitSeconds(1)
//                        //launchZoneToMiddleBalls
//                        .setReversed(false)
//                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(13, ballPickupYPos, pi/2), pi/2)
//                        .waitSeconds(1)
//                        .setTangent(pi/2)
                        .splineTo(ball2PickupPosition, pi/2) // slow mode
                        //middleBallsToLaunchZone
                        .setTangent(3*pi/2)
                        .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), pi)
                        //launchZoneToClassifier
                        .waitSeconds(0.5)
                        .setTangent(0)
//                        .splineToSplineHeading(new Pose2d(-4,30, 5*pi/8), pi/8)
                        .splineToSplineHeading(new Pose2d(12,58, 5*pi/8), pi/2)
                        .waitSeconds(0.5)
                        .setTangent(-pi/8)
                        //ClassifierToLaunchZone
                        .setTangent(3*pi/2)
                        .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), pi)
                        .waitSeconds(0.5)
                        //launchZoneToClassifier
                        .waitSeconds(0.5)
                        .setTangent(0)
//                        .splineToSplineHeading(new Pose2d(-4,30, 5*pi/8), pi/8)
                        .splineToSplineHeading(new Pose2d(12,58, 5*pi/8), pi/2)
                        .waitSeconds(0.5)
                        .setTangent(-pi/8)
                        //ClassifierToLaunchZone
                        .setTangent(3*pi/2)
                        .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), pi)
                        .waitSeconds(0.5)
//                        //launchZoneToFrontBalls
//                        .setReversed(false)
                        .setTangent(pi/2)
                        .splineToSplineHeading(new Pose2d(-11, ballPickupYPos, pi/2), pi/2)
//                        .setTangent(pi/2)
                        .splineTo(ball1PickupPosition, pi/2) // slow mode
//                        //frontBallsToLaunchZone
                        .setTangent(ball1ToEndLaunchAngle)
                        .splineToSplineHeading(new Pose2d(-34,35.8, 3*pi/4), ball1ToEndLaunchAngle)
                        .waitSeconds(0.5)
                        //autoEnd
                        .splineToSplineHeading(new Pose2d(endLaunchPosition, launchToGoalAngle), ball1ToEndLaunchAngle)

                        .build());
    }

    //transfered competition far code
    public static RoadRunnerBotEntity testPathFar(MeepMeep meepMeepFar) {
        Vector2d launchPosition = new Vector2d(56, 14.5);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-66, 58));

        double launchToSweepAngle = angleBetweenPoints(launchPosition, new Vector2d(58, 37));

        return new DefaultBotBuilder(meepMeepFar)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, 13.5, Math.toRadians(180)))
                        //startToLaunchZone
                        .splineTo(launchPosition, launchToGoalAngle)
                        .waitSeconds(0.5)
                        //startToHumanPlayer
                        //vertical*
//                        .splineToSplineHeading(new Pose2d(50, 25, pi/2), pi/2)
//                        .splineToSplineHeading(new Pose2d(50, 50, 3*pi/8), pi/2)
//                        .splineToSplineHeading(new Pose2d(50, 62, 0), pi/2)
                        //horizantal*
                        .setTangent(pi/2)
                        .splineToSplineHeading(new Pose2d(58,60, 3*pi/8), pi/2)
                        //humanPlayerToLaunch
                        .setReversed(true)
                        .setTangent(-pi/2)
                        .splineToConstantHeading(new Vector2d(56, 30), 3*pi/2)
                        .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), -pi/2)
                        .setTangent(launchToGoalAngle)
                        .waitSeconds(1)
                        .setReversed(false)
                        //launchZoneToSecondBalls
                        .splineTo(new Vector2d(36, 28), pi/2)
                        .splineTo(new Vector2d(36, 45), pi/2) // slow mode
                        //secondBallsToLaunchZone
                        .setReversed(true)
                        .splineTo(launchPosition, launchToGoalAngle - pi)
                        .waitSeconds(0.5)
//                        launchZoneToThirdBalls
                        .setReversed(false)
                        .splineTo(new Vector2d(13, 28), pi/2)
                        .splineTo(new Vector2d(13, 45), pi/2) // slow mode
                        //thirdBallsToLaunchZone
                        .setReversed(true)
                        .splineTo(launchPosition, launchToGoalAngle - pi)
                        .setReversed(false)
//                        sweep secret tunnel and human player area OPTION 1
//                        .setTangent(pi/2)
//                        .splineToSplineHeading(new Pose2d(58, 37, pi/2), launchToSweepAngle)
//                        .splineToSplineHeading(new Pose2d(54, 52, 3*pi/4), 5*pi/6)
//                        .splineToSplineHeading(new Pose2d(20, 60, pi), pi)

                        //launchToSweep
                        .splineToSplineHeading(new Pose2d(12,58, 5*pi/8), 3*pi/4)

                        //sweepToLaunchZone
                        .setReversed(true)
                        .splineTo(launchPosition, launchToGoalAngle - pi)
                        .waitSeconds(0.5)

                        //endAuto
                        .lineTo(new Vector2d(56, 30))

                        .build());
    }

    //starts on the right side, towards red goal
    public static RoadRunnerBotEntity testPathRightRed(MeepMeep meepMeepRightRed) {
        Vector2d launchPosition = new Vector2d(56, 14.5);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-66, 58));

        return new DefaultBotBuilder(meepMeepRightRed)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, 13.5, Math.toRadians(180)))
                        .splineTo(launchPosition, launchToGoalAngle)

                        .waitSeconds(3)

                        .splineTo(new Vector2d(36, 28), pi/2)
                        .splineTo(new Vector2d(36, 45), pi/2) // change to slow mode

                        .waitSeconds(3)

                        .setReversed(true).splineTo(launchPosition, launchToGoalAngle - pi)

                        .waitSeconds(3)

                        .setReversed(false)
                        .splineTo(new Vector2d(13, 28), pi/2)
                        .splineTo(new Vector2d(13, 45), pi/2) // change to slow mode

                        .waitSeconds(3)

                        .setReversed(true).splineTo(launchPosition, launchToGoalAngle - pi)

                        .waitSeconds(3)

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
        Vector2d launchPosition = new Vector2d(-25, 20);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-58, 58));

        Vector2d ball1PickupPosition = new Vector2d(-11, 55);
        double ball1ToLaunchAngle = angleBetweenPoints(ball1PickupPosition, launchPosition);

        Vector2d ball2PickupPosition = new Vector2d(13, 59);
        double ball2ToLaunchAngle = angleBetweenPoints(ball2PickupPosition, launchPosition);

        return new DefaultBotBuilder(meepMeepLeftRed)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, 39, Math.toRadians(180)))


                        .setReversed(true)
                        .splineTo(launchPosition, launchToGoalAngle - pi)

                        .waitSeconds(3)

                        .setReversed(false)
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(-11, 20, pi/2), 0)
                        .setTangent(pi/2)
                        .splineTo(ball1PickupPosition, pi/2)

                        .waitSeconds(3)
                        .setTangent(ball1ToLaunchAngle)
                        .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), ball1ToLaunchAngle)

                        .waitSeconds(3)

                        .setReversed(false)
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(13, 20, pi/2), 0)
                        .setTangent(pi/2)
                        .splineTo(ball2PickupPosition, pi/2)

                        .waitSeconds(3)

                        .setTangent(ball2ToLaunchAngle)
                        .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), ball2ToLaunchAngle)

                        .waitSeconds(3)

                        .build());
    }
    //starts on the right side, aims towards blue goal
    public static RoadRunnerBotEntity testPathRightBlue(MeepMeep meepMeepRightBlue) {
        Vector2d launchPosition = new Vector2d(54, -15);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-66, -58));

        return new DefaultBotBuilder(meepMeepRightBlue)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, -13.5, Math.toRadians(180)))
                        .splineTo(launchPosition, launchToGoalAngle)

                        .waitSeconds(3)

                        .splineTo(new Vector2d(36, -28),3*pi/2)
                        .splineTo(new Vector2d(36, -55), 3*pi/2) // change to slow mode

                        .waitSeconds(3)

                        .setReversed(true).splineTo(launchPosition, launchToGoalAngle - pi)

                        .waitSeconds(3)

                        .setReversed(false)
                        .splineTo(new Vector2d(13, -28), 3*pi/2)
                        .splineTo(new Vector2d(13, -55), 3*pi/2) // change to slow mode

                        .waitSeconds(3)

                        .setReversed(true).splineTo(launchPosition, launchToGoalAngle - pi)

                        .waitSeconds(3)

                        //.setReversed(false)
                        //.splineTo(new Vector2d(40, -17), launchToGoalAngle)
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
        Vector2d launchPosition = new Vector2d(-25, -25);
        double launchToGoalAngle = angleBetweenPoints(launchPosition, new Vector2d(-60, -58));

        Vector2d ball1PickupPosition = new Vector2d(-11, -55);
        double ball1ToLaunchAngle = angleBetweenPoints(ball1PickupPosition, launchPosition);

        Vector2d ball2PickupPosition = new Vector2d(13, -55);
        double ball2ToLaunchAngle = angleBetweenPoints(ball2PickupPosition, launchPosition);

        return new DefaultBotBuilder(meepMeepLeftBlue)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-61, -36, Math.toRadians(180)))

                        .setReversed(true)
                        .splineTo(launchPosition, launchToGoalAngle - pi)



                        .waitSeconds(3)

                        .setTangent(pi).splineToSplineHeading(new Pose2d(-58, -25, pi), pi)

                        .setReversed(false)
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(-11, -25, 3*pi/2), 0)
                        .setTangent(3*pi/2)
                        .splineTo(ball1PickupPosition, 3*pi/2)

                        .waitSeconds(3)
                        .setTangent(ball1ToLaunchAngle)
                        .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), ball1ToLaunchAngle)

                        .waitSeconds(3)

                        .setReversed(false)
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(13, -25, 3*pi/2), 0)
                        .setTangent(3*pi/2)
                        .splineTo(ball2PickupPosition, 3*pi/2)

                        .waitSeconds(3)

                        .setTangent(ball2ToLaunchAngle)
                        .splineToSplineHeading(new Pose2d(launchPosition, launchToGoalAngle), ball2ToLaunchAngle)

                        .waitSeconds(3)

//                        .setReversed(true).splineToLinearHeading(new Pose2d(-25, -27, -launchToGoalAngle - pi/8), -pi)
//
//                        .waitSeconds(3)
//                        .strafeLeft(15)
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


