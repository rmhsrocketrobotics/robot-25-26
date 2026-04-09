package org.firstinspires.ftc.teamcode.teamcode;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.CustomMath;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.State;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class DemonstrationAuto extends LinearOpMode {

    public abstract class DynamicTrajectoryAction implements Action {
        private boolean initialized = false;
        private Action trajectoryAction;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                trajectoryAction = createPath();
                initialized = true;
            }

            return trajectoryAction.run(packet);
        }

        public abstract Action createPath();
    }

    public class RecordPose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            PoseStorage.currentPose = drive.localizer.getPose();
            return true;
        }
    }

    /// always returns true
    public Action recordPose() {
        return new RecordPose();
    }

    public class PathGenerator {
        /// this just applies a vertical shift to the entire path (which does literally nothing)
        /// the only reason this is a thing is bc of limitations in meepmeep
        /// just dw
        private final double scrollConstant = 0;

        PathGenerator() {
            beginPose = new Pose2d(0, 60 + scrollConstant, Math.toRadians(90));
            drive = new MecanumDrive(hardwareMap, beginPose);
        }

        public Action startToBalls() {
            return drive.actionBuilder(beginPose)
                    .setTangent(3*pi/2)
                    .splineToConstantHeading(new Vector2d(0, 0 + scrollConstant), 3*pi/2)

                    .splineTo(new Vector2d(15, -30 + scrollConstant), 3*pi/2)
                    .splineTo(new Vector2d(0, -60 + scrollConstant), 5*pi/4)
                    .splineTo(new Vector2d(-15, -90 + scrollConstant), 3*pi/2)
                    .splineTo(new Vector2d(0, -120 + scrollConstant), 7*pi/4)

                    .splineTo(new Vector2d(15, -150 + scrollConstant), 3*pi/2)
                    .splineTo(new Vector2d(0, -180 + scrollConstant), 5*pi/4)
                    .splineTo(new Vector2d(-15, -210 + scrollConstant), 3*pi/2)
                    .splineTo(new Vector2d(0, -240 + scrollConstant), 7*pi/4)

                    .splineToSplineHeading(new Pose2d(0, -260 + scrollConstant, 3*pi/2), 3*pi/2)

                    .splineToConstantHeading(new Vector2d(0, -280 + scrollConstant), 3*pi/2)

                    .build();
        }

        public Action ballsToLaunch() {
            return drive.actionBuilder(beginPose)
                    // TODO code here

                    .build();
        }

        public Action launchToBalls() {
            return drive.actionBuilder(beginPose)
                    // TODO code here

                    .build();
        }

        public class UpdateLocalizer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drive.localizer.update();
                return true;
            }
        }

        public Action updateLocalizer() {
            return new UpdateLocalizer();
        }
    }

    public class BallHandler { // combination of spindex and outtake
        Spindex spindex;
        Outtake outtake;
        AutoVision vision;

        public BallHandler(HardwareMap hardwareMap) {
            spindex = new Spindex(hardwareMap);
            spindex.ballStates = new Spindex.BallState[]{Spindex.BallState.GREEN, Spindex.BallState.PURPLE, Spindex.BallState.PURPLE};

            outtake = new Outtake(hardwareMap);
            //outtake.tolerance = 100;
        }

        public void init() {
            spindex.init("outtake", 2);

            outtake.init();
        }

        public class TrackObelisk implements Action {
            final boolean trackObelisk;
            public TrackObelisk(boolean trackObelisk) {
                this.trackObelisk = trackObelisk;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vision.detectObelisk();
                telemetry.addData("obelisk id", vision.obeliskId);
                telemetry.update();
                return true;
            }
        }

        /// always returns true
        public Action trackObelisk(boolean trackObelisk) {
            return new TrackObelisk(trackObelisk);
        }

        public class LaunchAllBalls implements Action {
            private boolean initialized = false;
            private double launchDistance;
            private boolean autoFindLaunchDistance;
            private Vector2d goalPosition;
            private boolean sortBalls;

            public LaunchAllBalls(double launchDistance, boolean sortBalls) {
                this.sortBalls = sortBalls;

                if (launchDistance == 0) {
                    this.autoFindLaunchDistance = true;

                    goalPosition = new Vector2d(-58, 55); // TODO: change this to wherever the goal is

                    this.launchDistance = CustomMath.distanceBetweenPoints(drive.localizer.getPose().component1(), goalPosition) / 39.37;;

                } else {
                    this.autoFindLaunchDistance = false;
                    this.launchDistance = launchDistance;
                }
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    for (int i = 0; i < 3; i++) {
                        if (spindex.ballStates[i] == Spindex.BallState.EMPTY) {
                            spindex.ballStates[i] = Spindex.BallState.PURPLE;
                        }
                    }

                    spindex.intakeMotor.setPower(0.5);

                    if (!sortBalls) {
                        spindex.shootAllBalls();

                    } else if (vision.obeliskId == 21) {
                        spindex.queueBall("green");
                        spindex.queueBall("purple");
                        spindex.queueBall("purple");

                    } else if (vision.obeliskId == 22) {
                        spindex.queueBall("purple");
                        spindex.queueBall("green");
                        spindex.queueBall("purple");

                    } else if (vision.obeliskId == 23) {
                        spindex.queueBall("purple");
                        spindex.queueBall("purple");
                        spindex.queueBall("green");

                    } else {
                        spindex.shootAllBalls();
                    }

                    outtake.setOuttakeVelocityAndHoodAngle(launchDistance);
                }

                if (autoFindLaunchDistance) {
                    this.launchDistance = CustomMath.distanceBetweenPoints(drive.localizer.getPose().component1(), goalPosition) / 39.37;
                    // alr bro listen ik this is a stupid way to try to do moving while shooting but i can't be bothered and it might work
                    outtake.setOuttakeVelocityAndHoodAngle(launchDistance + 0.2);
                }

                spindex.update(outtake, State.OUTTAKE);
                outtake.update();

//                outtake.printTelemetry(telemetry);
//                spindex.printTelemetry(telemetry);
//                telemetry.update();

                PoseStorage.ballStates = spindex.ballStates;

                return !spindex.shouldSwitchToIntake;
            }
        }
        /// returns true until all balls are launched, then returns false
        public Action launchAllBalls(double launchDistance, boolean sortBalls) {
            return new LaunchAllBalls(launchDistance, sortBalls);
        }

        /// returns true until all balls are launched, then returns false
        public Action launchAllBalls(double launchDistance) {
            return new LaunchAllBalls(launchDistance, true);
        }

        public class RunActiveIntake implements Action {
            private boolean initialized = false;
            private boolean resetDrum;

            public RunActiveIntake(boolean resetDrum) {
                this.resetDrum = resetDrum;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    outtake.targetTicksPerSecond = 0;

                    if (this.resetDrum) {
                        spindex.recheckDrum();
                    }

                    spindex.intakeMotor.setPower(1);
                }
                spindex.update(outtake, State.INTAKE);
                outtake.update();

                PoseStorage.ballStates = spindex.ballStates;

                return true;//!spindex.shouldSwitchToOuttake;
            }
        }
        /// this method returns true until the drum is full; then it returns false
        ///
        /// ***EDIT: this method now always returns true***
        public Action runActiveIntake(boolean resetDrum) {
            return new RunActiveIntake(resetDrum);
        }

        public class ReadyOuttake implements Action {
            private boolean initialized = false;
            private double launchDistance;
            private boolean sortBalls;
            private ElapsedTime spitTimer;
            private double spitTime = 0.5;
            private boolean hasSpit = false;

            public ReadyOuttake(double launchDistance, boolean sortBalls) {
                this.launchDistance = launchDistance;
                this.sortBalls = sortBalls;
                this.spitTimer = new ElapsedTime(ElapsedTime.SECOND_IN_NANO);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;

                    if (!sortBalls) {
                        spindex.setDrumStateToNextOuttake();

                    } else if (vision.obeliskId == 21) {
                        spindex.setDrumStateToNextOuttake(Spindex.BallState.GREEN, true);

                    } else if (vision.obeliskId == 22 || vision.obeliskId == 23) {
                        spindex.setDrumStateToNextOuttake(Spindex.BallState.PURPLE, true);

                    } else {
                        spindex.setDrumStateToNextOuttake();
                    }

                    outtake.setOuttakeVelocityAndHoodAngle(launchDistance);
                }

                if (!hasSpit && !(spindex.switchCooldownTimer.seconds() < spindex.switchCooldown + 0.5)) {
                    spitTimer.reset();
                    hasSpit = true;
                }

                if (spitTimer.seconds() < spitTime) {
                    spindex.intakeMotor.setPower(-1);
                } else if (hasSpit) {
                    spindex.intakeMotor.setPower(0.5);
                } else {
                    spindex.intakeMotor.setPower(1);
                }

                spindex.update(outtake, State.OUTTAKE);
                outtake.update();

                PoseStorage.ballStates = spindex.ballStates;

                return true;
            }
        }

        /// always returns true
        public Action readyOuttake(double launchDistance, boolean sortBalls) {
            return new ReadyOuttake(launchDistance, sortBalls);
        }

        /// always returns true
        public Action readyOuttake(double launchDistance) {
            return new ReadyOuttake(launchDistance, true);
        }

        public class DrumIsEmpty implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return spindex.drumIsEmpty();
            }
        }

        /// keeps returning true as long as the drum is empty
        public Action drumIsEmpty() {
            return new DrumIsEmpty();
        }

        public class StopIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spindex.intakeMotor.setPower(0);
                return false;
            }
        }

        /// stops the intake (immediately returns false)
        public Action stopIntake() {
            return new StopIntake();
        }
    }

    public double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double x = point2.x - point1.x;
        double y = point2.y - point1.y;

        return Math.atan2(y, x);
    }

    public final double pi = Math.PI;

    Pose2d beginPose;
    MecanumDrive drive;

    VelConstraint lowVelocity;
    VelConstraint mediumVelocity;
    BallHandler ballHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        //Vision vision = new Vision(hardwareMap, allianceIsRed());
        ballHandler = new BallHandler(hardwareMap);

        lowVelocity = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 15;
            }
        };

        mediumVelocity = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 15;
            }
        };

        runAutoBlocking();
    }

    public void runAutoBlocking() {
        PathGenerator path = new PathGenerator();

        // TODO: add time it takes to go through the path so the intake only activates right before getting to the balls
        double intakeActivationDelay = 10;

        /// action to launch preload, go through the obstacle course, then pick up the balls
        Action startToBalls = new SequentialAction(
                new RaceAction(
                        path.startToBalls(),

                        new SequentialAction(
                                ballHandler.launchAllBalls(0, false),
                                new SleepAction(intakeActivationDelay),
                                ballHandler.runActiveIntake(true)
                        )
                )
        );

        /// action to go from the balls to the launch zone back to the balls
        /// this action can be repeated as many times as we want
        Action ballsToLaunchLoop = new SequentialAction(
                ballHandler.stopIntake(),
                path.ballsToLaunch(),
                ballHandler.launchAllBalls(0, false),

                new RaceAction(
                        path.launchToBalls(),

                        new SequentialAction(
                                new SleepAction(intakeActivationDelay),
                                ballHandler.runActiveIntake(true)
                        )
                )
        );

        /// last action that the robot will do
        Action ballsToLaunchLoopEnd = new SequentialAction(
                ballHandler.stopIntake(),
                path.ballsToLaunch(),
                ballHandler.launchAllBalls(0, false)
        );

        Action autonomous = new SequentialAction(
                startToBalls,
                ballsToLaunchLoop,
                ballsToLaunchLoopEnd
        );

        ballHandler.init();

        waitForStart();

        Actions.runBlocking(autonomous);
    }
}