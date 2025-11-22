///* Copyright (c) 2023 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.LastYearExampleCode;
//
//import android.util.Size;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
//import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import java.lang.Math;
//import java.util.List;
//
///*
// * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
// * including Java Builder structures for specifying Vision parameters.
// *
// * For an introduction to AprilTags, see the FTC-DOCS link below:
// * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
// *
// * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
// * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
// * the current Season's AprilTags and a small set of "test Tags" in the high number range.
// *
// * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
// * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
// * https://ftc-docs.firstinspires.org/apriltag-detection-values
// *
// * To experiment with using AprilTags to navigate, try out these two driving samples:
// * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
// *
// * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
// * These default parameters are shown as comments in the code below.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
// */
//@TeleOp(name = "Concept: AprilTag", group = "Concept")
//
//public class ConceptAprilTag extends LinearOpMode {
//
//    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//    /**
//     * The variable to store our instance of the AprilTag processor.
//     */
//    private AprilTagProcessor aprilTag;
//
//    /**
//     * The variable to store our instance of the vision portal.
//     */
//    private VisionPortal visionPortal;
//
//    @Override
//    public void runOpMode() {
//
//        DcMotor brMotor = hardwareMap.get(DcMotor.class, "brMotor");
//        DcMotor blMotor = hardwareMap.get(DcMotor.class, "blMotor");
//        DcMotor frMotor = hardwareMap.get(DcMotor.class, "frMotor");
//        DcMotor flMotor = hardwareMap.get(DcMotor.class, "flMotor");
//
//        flMotor.setDirection(DcMotor.Direction.REVERSE);
//        blMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        initAprilTag();
//
//        // Wait for the DS start button to be touched.
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch START to start OpMode");
//        telemetry.update();
//        waitForStart();
//        double bearing = 0.1;
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//
//                List<AprilTagDetection> detections = telemetryAprilTag();
//
//                if (!detections.isEmpty() && detections.get(0).metadata != null) {
//                    AprilTagDetection detection = detections.get(0);
//                    bearing = detection.ftcPose.bearing * 0.05;
//                    if (detection.ftcPose.bearing < 10.0 && detection.ftcPose.bearing > -10.0){
//                        bearing = detection.ftcPose.bearing * 0.02;
//                    }
//                    double clampValue = 0.3;
//                    flMotor.setPower(clamp(bearing, -clampValue, clampValue));
//                    blMotor.setPower(clamp(bearing, -clampValue, clampValue));
//                    frMotor.setPower(clamp(-bearing, -clampValue, clampValue));
//                    brMotor.setPower(clamp(-bearing, -clampValue, clampValue));
//
//                } else {
//                    double newBearing;
//                    double clampValue = 0.3;
//                    if (bearing > 0){
//                        newBearing = clampValue;
//                    } else if (bearing < 0){
//                        newBearing = -clampValue;
//                    } else {
//                        newBearing = 0;
//                    }
//                    flMotor.setPower(clamp(bearing, -clampValue, clampValue));
//                    blMotor.setPower(clamp(bearing, -clampValue, clampValue));
//                    frMotor.setPower(clamp(-bearing, -clampValue, clampValue));
//                    brMotor.setPower(clamp(-bearing, -clampValue, clampValue));
//                }
//
//                // Push telemetry to the Driver Station.
//                telemetry.update();
//
//                // Save CPU resources; can resume streaming when needed.
//                if (gamepad1.dpad_down) {
//                    visionPortal.stopStreaming();
//                } else if (gamepad1.dpad_up) {
//                    visionPortal.resumeStreaming();
//                }
//
//                // Share the CPU.
//                sleep(20);
//
//
//            }
//        }
//
//        // Save more CPU resources when camera is no longer needed.
//        visionPortal.close();
//
//    }   // end method runOpMode()
//
//    /**
//     * Initialize the AprilTag processor.
//     */
//    private void initAprilTag() {
//        AprilTagMetadata myAprilTagMetadata;
//        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
//        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
//        AprilTagLibrary myAprilTagLibrary;
//        AprilTagProcessor myAprilTagProcessor;
//
//// Create a new AprilTagLibrary.Builder object and assigns it to a variable.
//        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
//
//// Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
//// Get the AprilTagLibrary for the current season.
//        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
//
//// Create a new AprilTagMetdata object and assign it to a variable.
//        myAprilTagMetadata = new AprilTagMetadata(20, "please work i beg", 6.5, DistanceUnit.INCH);
//
//// Add a tag to the AprilTagLibrary.Builder.
//        myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);
//
//// Build the AprilTag library and assign it to a variable.
//        myAprilTagLibrary = myAprilTagLibraryBuilder.build();
//
//// Create a new AprilTagProcessor.Builder object and assign it to a variable.
//        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//
//// Set the tag library.
//        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
//
//// Build the AprilTag processor and assign it to a variable.
//        aprilTag = myAprilTagProcessorBuilder.build();
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//        // Set and enable the processor.
//        builder.addProcessor(aprilTag);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        // Disable or re-enable the aprilTag processor at any time.
//        //visionPortal.setProcessorEnabled(aprilTag, true);
//
//    }   // end method initAprilTag()
//
//
//    /**
//     * Add telemetry about AprilTag detections.
//     */
//    private List<AprilTagDetection> telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//
//
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//
//        return currentDetections;
//    }   // end method telemetryAprilTag()
//
//    static double clamp(double value, double min, double max) {
//        return Math.min(Math.max(min, value), max);
//    }
//}   // end class
//
