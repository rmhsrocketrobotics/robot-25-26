package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Velocity;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Vision;

// this class is literally just a copy of mainteleop, but it has been changed to test the shooter
// i deleted like most of the normal teleop code from here dw its fine probably
@TeleOp
public class LaunchTesting extends LinearOpMode{
    String state;
    Drivetrain drivetrain;
    Spindex spindex;
    Outtake outtake;
    Vision vision;
    Odometry odometry;
    Gamepad gamepad1Last;
    Gamepad gamepad2Last;

    boolean pauseIntake = false;
    double hoodServoPosition = 0;
    double outtakeSpeed = 1000;

    public boolean allianceIsRed() {
        return true;
    }

    @Override
    public void runOpMode() {
        state = "intake"; // states are: "intake", "transition", and "outtake"

        drivetrain = new Drivetrain(hardwareMap); // wheels
        spindex = new Spindex(hardwareMap, false); // drumServo, intake, flick
        spindex.flickTime = 1;
        spindex.postFlickTime = 0.4;

        outtake = new Outtake(hardwareMap); // outtake, hoodServo
        vision = new Vision(hardwareMap, allianceIsRed()); // camera
        odometry = new Odometry(hardwareMap); // pinpoint

        gamepad1Last = new Gamepad();
        gamepad2Last = new Gamepad();

        telemetry.setMsTransmissionInterval(100); //default 250

        waitForStart();

        spindex.init();

        while (opModeIsActive()) {

            /// GAMEPAD 1 CODE
            if (gamepad1.dpad_up && !gamepad1Last.dpad_up) {
                pauseIntake = !pauseIntake;
            }

            if (gamepad1.right_bumper && !gamepad1Last.right_bumper) {
                hoodServoPosition += 0.05;
            } else if (gamepad1.left_bumper && !gamepad1Last.left_bumper) {
                hoodServoPosition -= 0.05;
            }
            outtake.hoodServo.setPosition(hoodServoPosition);

            if (gamepad1.x && !gamepad1Last.x) {
                outtakeSpeed += 100;
            } else if (gamepad1.y && !gamepad1Last.y) {
                outtakeSpeed -= 100;
            }
            outtake.targetTicksPerSecond = outtakeSpeed;

            if (gamepad1.dpad_right && !gamepad1Last.dpad_right) {
                outtake.tolerance += 10;
            } else if (gamepad1.dpad_left && !gamepad1Last.dpad_left) {
                outtake.tolerance -= 10;
            }

            gamepad1Last.copy(gamepad1);


            /// GAMEPAD 2 CODE
            if (gamepad2.right_bumper && !gamepad2Last.right_bumper) {
                spindex.switchCooldownConstant += 0.1;
            } else if (gamepad2.left_bumper && !gamepad2Last.left_bumper) {
                spindex.switchCooldownConstant -= 0.1;
            }

            if (gamepad2.x && !gamepad2Last.x) {
                spindex.flickTime += 0.1;
            } else if (gamepad2.y && !gamepad2Last.y) {
                spindex.flickTime -= 0.1;
            }

            if (gamepad2.dpad_right && !gamepad2Last.dpad_right) {
                spindex.postFlickTime += 0.1;
            } else if (gamepad2.dpad_left && !gamepad2Last.dpad_left) {
                spindex.postFlickTime -= 0.1;
            }

            gamepad2Last.copy(gamepad2);

            telemetry.addData("distance to the goal is", vision.goalDistance);

            telemetry.addLine();

            telemetry.addData("hood servo position is", hoodServoPosition);
            telemetry.addData("outtake speed is", outtakeSpeed);
            telemetry.addData("outtake tolerance is", outtake.tolerance);

            telemetry.addLine();

            telemetry.addData("the switch cooldown constant is", spindex.switchCooldownConstant);
            telemetry.addData("the flick time is", spindex.flickTime);
            telemetry.addData("the post flick time is", spindex.postFlickTime);

            telemetry.addLine("\n---------------------------\n");

            // state specific code goes in these methods
            if (state.equals("intake")) {
                intakeMode();
            } else if (state.equals("outtake")) {
                outtakeMode();
            }

            spindex.update(outtake);
            outtake.update(spindex);
            vision.update(odometry.currentBearing);
            odometry.update();

//            outtake.printTelemetry(telemetry);
//            vision.printTelemetry(telemetry);
            telemetry.addData("state", state);
            telemetry.update();
        }
    }

    public void intakeMode() {
        if (spindex.shouldSwitchToOuttake) {
            state = "outtake";
            spindex.setDrumState("outtake", 0);
            //vision.seenGoalAprilTag = false;
            spindex.ballQueue.clear();

            spindex.queueBall("green");
            spindex.queueBall("green");
            spindex.queueBall("green");

            spindex.intake.setPower(0);

            return;
        }
        if (pauseIntake) {
            spindex.intake.setPower(1);
        } else {
            spindex.intake.setPower(0);
        }
    }

    public void outtakeMode() {
        if (spindex.shouldSwitchToIntake) {
            state = "intake";
            spindex.setDrumState("intake", 0);

            return;
        }
    }
}