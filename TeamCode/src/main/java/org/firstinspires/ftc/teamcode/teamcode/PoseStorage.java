package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0, 0, 0);
    public static Spindex.BallState[] ballStates = {Spindex.BallState.EMPTY, Spindex.BallState.EMPTY, Spindex.BallState.EMPTY};
}
