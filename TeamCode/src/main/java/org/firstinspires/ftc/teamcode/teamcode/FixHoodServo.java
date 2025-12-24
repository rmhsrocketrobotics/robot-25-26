package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "!main")

public class FixHoodServo extends LinearOpMode{

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "hoodServo");

        telemetry.setMsTransmissionInterval(200); //default 250

        waitForStart();


        while (opModeIsActive()) {
            servo.setPosition((gamepad2.left_stick_y / 2) + 0.5);
            telemetry.addData("hood servo position", servo.getPosition());
            telemetry.update();
        }
    }
}