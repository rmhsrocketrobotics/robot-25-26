package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import android.util.Size;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.lang.Math;
import java.util.List;
@TeleOp

public class TeleOpModes extends LinearOpMode {
    ElapsedTime switchTimer = new ElapsedTime();
    ServoImplEx drumServo;
    DcMotor intake;
    NormalizedColorSensor colorSensor;
    String color;
    String[] ballStates = {"none", "none", "none"};
    int drumPos;
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode(){
        drumServo = hardwareMap.get(ServoImplEx.class, "drumServo");
        drumServo.setPwmRange(new PwmRange(500, 2500));

        intake = hardwareMap.get(DcMotor.class, "intake");

        // set color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorLeft");


        //turn on light
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        //set gain
        colorSensor.setGain(15);
        waitForStart();
        drumPos = 0;
        //was button pressed last loop?
        boolean lastAButton = false;
        while (opModeIsActive()){
            double position = -gamepad1.left_stick_y;
            if (detectColor()) {
                drumPos += 1;
                switchTimer.reset();
            }
            if (!lastAButton && gamepad1.a){
                drumPos = drumPos + 1;
                if (drumPos > 5){
                    drumPos = 5;
                }
                switchTimer.reset();

            }
            intake.setPower(-gamepad1.left_stick_y);

            lastAButton = gamepad1.a;
            if (gamepad1.b){
                drumPos = 0;
            }
            if (drumPos<3){
                intakeDrum(drumPos);
            } else {
                outtakeDrum(drumPos % 3);
            }
            telemetry.update();
        }

    }
    public boolean detectColor(){
        if (switchTimer.seconds() < 1) {
            return false;
        }
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        String color = "unknown";

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0]);
        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));

        if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 3 && ballStates[drumPos] == "none"){
            if (hsvValues[0] > 200) {
                color = "purple";
            } else if (hsvValues[0] < 180) {
                color = "green";
            }
            ballStates[drumPos] = color;
            //telemetry.addData("ball detected! color", color);
            return true;
        }
        telemetry.addData("ballStates0", ballStates[0]);
        telemetry.addData("ballStates1", ballStates[1]);
        telemetry.addData("ballStates2", ballStates[2]);
        return false;
    }
    public void intakeDrum(int ballSlot) {
        double[] positions = {0, 0.3826, 0.7831};
        drumServo.setPosition(positions[ballSlot]);
    }
    public void outtakeDrum(int ballSlot) {
        double[] positions = {0.5745, 0.95, 0.1823};
        drumServo.setPosition(positions[ballSlot]);
    }
    public class Vision {
        public void aprilTags(){

            initAprilTag();

            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch START to start OpMode");
            telemetry.update();
            waitForStart();
            double bearing = 0.1;

            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    List<AprilTagDetection> detections = telemetryAprilTag();

                    if (!detections.isEmpty() && detections.get(0).metadata != null) {
                        AprilTagDetection detection = detections.get(0);
                        bearing = detection.ftcPose.bearing * 0.05;
                        if (detection.ftcPose.bearing < 10.0 && detection.ftcPose.bearing > -10.0){
                            bearing = detection.ftcPose.bearing * 0.02;
                        }
                        double clampValue = 0.3;

                    } else {
                        double newBearing;
                        double clampValue = 0.3;
                        if (bearing > 0){
                            newBearing = clampValue;
                        } else if (bearing < 0){
                            newBearing = -clampValue;
                        } else {
                            newBearing = 0;
                        }
                    }
                    // Push telemetry to the Driver Station.
                    telemetry.update();

                    // Save CPU resources; can resume streaming when needed.
                    if (gamepad1.dpad_down) {
                        visionPortal.stopStreaming();
                    } else if (gamepad1.dpad_up) {
                        visionPortal.resumeStreaming();
                    }

                    // Share the CPU.
                    sleep(20);


                }
            }
            visionPortal.close();
        }
        public void obelisk(){

        }
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        AprilTagMetadata myAprilTagMetadata;
        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagLibrary myAprilTagLibrary;
        AprilTagProcessor myAprilTagProcessor;

// Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

// Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
// Get the AprilTagLibrary for the current season.
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

// Create a new AprilTagMetdata object and assign it to a variable.
        myAprilTagMetadata = new AprilTagMetadata(20, "please work i beg", 6.5, DistanceUnit.INCH);

// Add a tag to the AprilTagLibrary.Builder.
        myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);

// Build the AprilTag library and assign it to a variable.
        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

// Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

// Set the tag library.
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

// Build the AprilTag processor and assign it to a variable.
        aprilTag = myAprilTagProcessorBuilder.build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private List<AprilTagDetection> telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }


        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


        return currentDetections;
    }   // end method telemetryAprilTag()

    static double clamp(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
    }
}   // end class


