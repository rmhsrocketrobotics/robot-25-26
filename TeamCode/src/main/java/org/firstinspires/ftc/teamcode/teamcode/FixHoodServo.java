package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Velocity;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(group = "!main")

public class FixHoodServo extends LinearOpMode{

    Outtake outtake;

    @Override
    public void runOpMode() {
        outtake = new Outtake(hardwareMap); // outtake, hoodServo

        telemetry.setMsTransmissionInterval(200); //default 250

        waitForStart();


        while (opModeIsActive()) {
            outtake.hoodServo.setPosition((gamepad2.left_stick_y / 2) + 0.5);
            telemetry.addData("hood servo position", outtake.hoodServo.getPosition());
            telemetry.update();
        }
    }
}