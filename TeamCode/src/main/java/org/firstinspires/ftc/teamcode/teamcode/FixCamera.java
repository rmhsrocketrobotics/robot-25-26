package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.subsystems.Vision;

@TeleOp
public class FixCamera extends LinearOpMode {
    Vision vision;

    public boolean allianceIsRed() {
        return true;
    }

    @Override
    public void runOpMode() {
        vision = new Vision(hardwareMap, allianceIsRed()); // camera

        telemetry.setMsTransmissionInterval(200); //default 250

        waitForStart();

        while (opModeIsActive()) {
            vision.update();

            telemetry.addData("pitch error", vision.findPitchError());
            vision.printTelemetry(telemetry);

            telemetry.update();
        }
    }
}