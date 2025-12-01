package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Velocity;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(group = "!main")

public class MainTeleop extends LinearOpMode{
    String state;
    Drivetrain drivetrain;
    Spindex spindex;
    Outtake outtake;
    Vision vision;
    Odometry odometry;
    Gamepad gamepad1Last;
    Gamepad gamepad2Last;

    @Override
    public void runOpMode() {
        state = "intake"; // states are: "intake", "transition", and "outtake"

        drivetrain = new Drivetrain(hardwareMap); // wheels
        spindex = new Spindex(hardwareMap); // drumServo, intake, flick
        outtake = new Outtake(hardwareMap); // outtake, hoodServo
        vision = new Vision(hardwareMap, false); // camera
        odometry = new Odometry(hardwareMap); // pinpoint

        gamepad1Last = new Gamepad();
        gamepad2Last = new Gamepad();

//        //random ahh imu stuff dw
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        imu.initialize(parameters);
//        imu.resetYaw();

        telemetry.setMsTransmissionInterval(200); //default 250
        //telemetry.setNumDecimalPlaces(0, 5);

//        //this is in place of a waitForStart() call
//        while (opModeInInit() && !isStopRequested()) {
//            vision.detectObelisk();
//        }

        waitForStart();

        spindex.init();

        while (opModeIsActive()) {
            // GAMEPAD 1 CODE:
            if (gamepad1.y && vision.seenGoalAprilTag) {
                vision.faceGoal(drivetrain, odometry.currentBearing);
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
//            if (gamepad2.x && !gamepad2Last.x) {
//                spindex.setDrumState("intake", 0);
//            }

            spindex.intake.setPower(gamepad2.right_trigger);
            //spindex.flick.setPower(gamepad2.left_trigger);

            Velocity requiredVelocity = vision.getRequiredVelocity();
            telemetry.addData("target speed m/s", requiredVelocity.speed);
            telemetry.addData("target angle degrees", requiredVelocity.direction);

            outtake.setHoodServoToAngle(requiredVelocity.direction);

            // state specific code goes in these methods
            if (state.equals("intake")) {
                intakeMode();
            } else if (state.equals("outtake")) {
                outtakeMode(requiredVelocity);
            }

            gamepad2Last.copy(gamepad2);

            spindex.update(outtake);
            outtake.update();
            vision.update(odometry.currentBearing);
            odometry.update();

            //drivetrain.printTelemetry(telemetry);
            outtake.printTelemetry(telemetry);
            vision.printTelemetry(telemetry);
            telemetry.addData("state", state);
            telemetry.update();
        }
    }

    public void intakeMode() {
        if ((gamepad2.dpad_up && !gamepad2Last.dpad_up) || spindex.shouldSwitchToOuttake) {
            state = "outtake";
            spindex.setDrumState("outtake", 0);
            //vision.seenGoalAprilTag = false;
            spindex.ballQueue.clear();
            return;
        } else if (gamepad2.dpad_down && !gamepad2Last.dpad_down) {
            spindex.setDrumState("intake", 0);
            return;
        }
    }

    public void outtakeMode(Velocity requiredVelocity) {
        if ((gamepad2.dpad_down && !gamepad2Last.dpad_down) || spindex.shouldSwitchToIntake) {
            state = "intake";
            spindex.setDrumState("intake", 0);
            outtake.targetTicksPerSecond = 0;
            return;
        }

        //vision.detectGoalAprilTag();

        outtake.setOuttakeToSpeed(requiredVelocity.speed);

        if (gamepad2.left_bumper && !gamepad2Last.left_bumper) {
            spindex.queueBall("purple");
        }
        if (gamepad2.right_bumper && !gamepad2Last.right_bumper) {
            spindex.queueBall("green");
        }
    }
}