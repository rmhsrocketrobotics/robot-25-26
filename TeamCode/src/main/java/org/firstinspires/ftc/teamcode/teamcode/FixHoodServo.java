package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(group = "!main")

public class FixHoodServo extends LinearOpMode{

    @Override
    public void runOpMode() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "drumServo");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

//        Servo servo = hardwareMap.get(Servo.class, "rBrake");

        double[] presetPositions = {0.25, 0.5, 0.75};
        double servoPosition = 0;

        telemetry.setMsTransmissionInterval(200); //default 250

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.y) {
                servoPosition = presetPositions[0];
            }
            if (gamepad1.x) {
                servoPosition = presetPositions[1];
            }
            if (gamepad1.a) {
                servoPosition = presetPositions[2];
            }

            if (gamepad1.rightBumperWasPressed()) {
                servoPosition += 0.01;
            } else if (gamepad1.leftBumperWasPressed()) {
                servoPosition -= 0.01;
            }

            servo.setPosition(servoPosition);
            telemetry.addData("servo position", servo.getPosition());
            telemetry.update();
        }
    }
}