package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "!main")

public class FixHoodServo extends LinearOpMode{

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "drumServo");

        double[] outtakePositions = {0.5845, 1, 0.15};
        double servoPosition = 0;

        telemetry.setMsTransmissionInterval(200); //default 250

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.y) {
                servoPosition = outtakePositions[0];
            }
            if (gamepad1.x) {
                servoPosition = outtakePositions[1];
            }
            if (gamepad1.a) {
                servoPosition = outtakePositions[2];
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