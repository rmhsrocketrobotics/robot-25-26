package org.firstinspires.ftc.teamcode.LastYearExampleCode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagTest extends LinearOpMode {
    @Override
    public void runOpMode(){

        //set settings for the processor here
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(CameraName.class, "camera"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        while (opModeIsActive()){
            // the following is what each id of the april tags mean:
            // navigation:
            // 20: blue alliance goal
            // 24: red alliance goal

            // codes:
            // 21, 22, 23: green first, green middle, green last

            for (AprilTagDetection tag : tagProcessor.getDetections()){
                switch (tag.id) {
                    case 20:
                        telemetry.addLine("blue alliance goal");
                        break;

                    case 21:
                        telemetry.addLine("green, purple, purple");
                        break;

                    case 22:
                        telemetry.addLine("purple, green, purple");
                        break;

                    case 23:
                        telemetry.addLine("purple, purple, green");
                        break;

                    case 24:
                        telemetry.addLine("red alliance goal");

                    default:
                        telemetry.addLine("tag with id " + tag.id + " detected (this shouldn't happen");
                }

            }
        }
    }
}
