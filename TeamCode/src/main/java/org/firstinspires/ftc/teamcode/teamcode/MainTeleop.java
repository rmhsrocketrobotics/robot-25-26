package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.State;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Drivetrain;

public class MainTeleop extends LinearOpMode {
    State state;
    Drivetrain drivetrain;
    Spindex spindex;
    Outtake outtake;
    Vision vision;
    Gamepad gamepad1Last;
    Gamepad gamepad2Last;

    boolean hasSpit;

    boolean pause;
//    LoopTimer loopTimer = new LoopTimer();

    public boolean allianceIsRed() {
        return true;
    }

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        state = State.INTAKE;

        drivetrain = new Drivetrain(hardwareMap); // wheels
        spindex = new Spindex(hardwareMap); // drumServo, intake, flick

        outtake = new Outtake(hardwareMap); // outtake, hoodServo
        vision = new Vision(hardwareMap, allianceIsRed()); // camera

        gamepad1Last = new Gamepad();
        gamepad2Last = new Gamepad();

        pause = false;

        ElapsedTime spitTimer = new ElapsedTime();
        double spitTime = 0.5;
        hasSpit = true;

//        //random ahh imu stuff dw
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        imu.initialize(parameters);
//        imu.resetYaw();

//        telemetry.setMsTransmissionInterval(200); //default 250
        //telemetry.setNumDecimalPlaces(0, 5);

//        //this is in place of a waitForStart() call
//        while (opModeInInit() && !isStopRequested()) {
//            vision.detectObelisk();
//        }

        waitForStart();

        spindex.init();
        outtake.init();
        vision.init();

        while (opModeIsActive()) {
            // GAMEPAD 1 CODE:
            if (gamepad1.y && vision.seenGoalAprilTag) {
                vision.faceGoal(drivetrain, telemetry);
            } else {
                if (gamepad1.left_bumper) {
                    // speed mode
                    drivetrain.setDrivetrainPower(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
                } else if (gamepad1.right_bumper) {
                    // slow mode
                    drivetrain.setDrivetrainPower(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x,
                            0.2, 0.2, 0.2);
                } else {
                    // normal mode
                    drivetrain.setDrivetrainPower(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x,
                            0.7, 0.7, 0.7);
                }
            }

//            outtake.hoodServo.setPosition((gamepad2.left_stick_y / 2) + 0.5);
//            telemetry.addData("servo position", outtake.hoodServo.getPosition());

            // literally all of the rest of this code is for gamepad 2:
            //spindex.flick.setPower(gamepad2.left_trigger);

//            if (gamepad2.y && !gamepad2Last.y) {
//                spindex.incrementDrumPosition();
//            }
//            if (gamepad2.x) {
//                spindex.flick.setPower(1);
//            }
//            if (gamepad2.x && !gamepad2Last.x) {
//                spindex.setDrumState("intake", 0);
//            }

            if (!hasSpit && !(spindex.switchCooldownTimer.seconds() < spindex.switchCooldown + 0.3)) {
                spitTimer.reset();
                hasSpit = true;
            }

            if (gamepad2.left_trigger > 0.2) { // backspin intake
                spindex.intakeMotor.setPower(-gamepad2.left_trigger);

            } else if ((gamepad2.right_trigger > 0.2)) { // spin intake
                spindex.intakeMotor.setPower(gamepad2.right_trigger);

            } else {
                if (!pause) {
                    if (spitTimer.seconds() < spitTime) {
                        spindex.intakeMotor.setPower(-1);
                    } else {
                        spindex.intakeMotor.setPower(0.5);
                    }

                } else {
                    spindex.intakeMotor.setPower(0);
                }
            }

            if (gamepad2.leftStickButtonWasPressed()) {
                pause = !pause;
            }

            // state specific code goes in these methods
            switch (state) {
                case INTAKE:
                    intakeMode();
                    break;

                case OUTTAKE:
                    outtakeMode();
                    break;
            }

//            if (gamepad2.y) {
//                outtake.setOuttakePower(1);
//            } else if (gamepad2.x) {
//                outtake.setOuttakePower(0.5);
//            } else if (gamepad2.a) {
//                outtake.setOuttakePower(0);
//            }

            gamepad2Last.copy(gamepad2);

            drivetrain.update();
            spindex.update(outtake, state);
            outtake.update();
            vision.update();

            //drivetrain.printTelemetry(telemetry);
//            spindex.printTelemetry(telemetry);
//            outtake.printTelemetry(telemetry);
//            vision.printTelemetry(telemetry);
//            if (state == State.INTAKE) {
//                telemetry.addLine("state: intake");
//            } else {
//                telemetry.addLine("state: outtake");
//            }

            //telemetry.addData("loops per second", Math.round(1 / loopTimer.getLoopTimeSeconds()));

            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), vision.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    public void intakeMode() {
        if ((gamepad2.dpad_up && !gamepad2Last.dpad_up) || spindex.shouldSwitchToOuttake) {
            state = State.OUTTAKE;
            hasSpit = false;

        } else if (gamepad2.dpad_down && !gamepad2Last.dpad_down) {
            spindex.recheckDrum();
        }
    }

    public void outtakeMode() {
        if ((gamepad2.dpad_down && !gamepad2Last.dpad_down) || spindex.shouldSwitchToIntake) {
            state = State.INTAKE;

            outtake.targetTicksPerSecond = 0;
            return;
        }

        if (pause) {
            outtake.targetTicksPerSecond = 0;
        } else {
            outtake.setOuttakeVelocityAndHoodAngle(vision.goalDistance);
        }


        if (gamepad2.left_bumper && !gamepad2Last.left_bumper) {
            spindex.queueBall("purple");
        }
        if (gamepad2.right_bumper && !gamepad2Last.right_bumper) {
            spindex.queueBall("green");
        }
        if (gamepad2.y) {
            spindex.shootAllBalls();
        }
    }
}