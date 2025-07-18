// code copied according to https://github.com/rh-robotics/MeepMeep/blob/master/INSTALL.md

package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static final double pi = Math.PI;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = autoV2(meepMeep);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static RoadRunnerBotEntity autoV1(MeepMeep meepMeep) {
        Pose2d bucketPose = new Pose2d(54, 58, 3*pi/4);
        Pose2d leftBlockPose = new Pose2d(48, 44, 3*pi/2);
        Pose2d middleBlockPose = new Pose2d(59, 44, 3*pi/2);

        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(38, 62, pi/2))
                        .waitSeconds(1)

                        .lineToSplineHeading(bucketPose)
                        .addDisplacementMarker(() -> {
                            //hypothetical code here
                        })
                        .waitSeconds(1) //depositing sample
                        .splineToSplineHeading(leftBlockPose, leftBlockPose.getHeading())
                        .waitSeconds(1) //picking up sample
                        .lineToSplineHeading(bucketPose)
                        .waitSeconds(1) //depositing sample
                        .lineToSplineHeading(middleBlockPose)
                        .waitSeconds(1) //picking up sample
                        .lineToSplineHeading(bucketPose)
                        .waitSeconds(1) //depositing sample

                        .lineToSplineHeading(new Pose2d(50, 50, 3*pi/2))
                        .splineToSplineHeading(new Pose2d(61, 10), 0)
                        .lineTo(new Vector2d(61, 45))
                        .splineToConstantHeading(new Vector2d(40, 12), 0)
                        .lineTo(new Vector2d(25, 12))

                        .waitSeconds(2)
                        .build());
    }

    public static RoadRunnerBotEntity autoV2(MeepMeep meepMeep) {
        Pose2d bucketPose = new Pose2d(54, 58, 3*pi/4);
        Pose2d leftBlockPose = new Pose2d(48, 44, 3*pi/2);
        Pose2d middleBlockPose = new Pose2d(59, 44, 3*pi/2);

        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(38, 62, pi/2))
                        .waitSeconds(1)

                        .splineToConstantHeading(new Vector2d(45, 45), 0)
                        .splineToLinearHeading(bucketPose, bucketPose.getHeading())

//                        .waitSeconds(1) //depositing sample
//                        .splineToSplineHeading(leftBlockPose, leftBlockPose.getHeading())
//                        .waitSeconds(1) //picking up sample
//                        .lineToSplineHeading(bucketPose)
//                        .waitSeconds(1) //depositing sample
//                        .lineToSplineHeading(middleBlockPose)
//                        .waitSeconds(1) //picking up sample
//                        .lineToSplineHeading(bucketPose)
//                        .waitSeconds(1) //depositing sample
//
//                        .lineToSplineHeading(new Pose2d(50, 50, 3*pi/2))
//                        .splineToSplineHeading(new Pose2d(61, 10), 0)
//                        .lineTo(new Vector2d(61, 45))
//                        .splineToConstantHeading(new Vector2d(40, 12), 0)
//                        .lineTo(new Vector2d(25, 12))

                        .waitSeconds(2)
                        .build());
    }

    public static RoadRunnerBotEntity exampleTrajectory(MeepMeep meepMeep) {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(38, 62, pi/2))

                    .waitSeconds(3)
                    .build());
    }
}