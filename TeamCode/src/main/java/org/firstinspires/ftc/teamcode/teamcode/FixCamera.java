package org.firstinspires.ftc.teamcode.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.subsystems.Vision;

@TeleOp
public class FixCamera extends LinearOpMode {
    Vision vision;
    NormalizedColorSensor intakeColorSensor;

    public boolean allianceIsRed() {
        return true;
    }

    @Override
    public void runOpMode() {
        vision = new Vision(hardwareMap, allianceIsRed()); // camera

        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
        intakeColorSensor.setGain(15);

        telemetry.setMsTransmissionInterval(200); //default 250

        waitForStart();

        vision.init();

        while (opModeIsActive()) {
            vision.update();

            telemetry.addData("pitch error", vision.findPitchError());
            telemetry.addData("color sensor distance", ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM));
            vision.printTelemetry(telemetry);

            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
            final float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addData("color", hsvValues[0]);

            telemetry.update();
        }
    }
}