//package org.firstinspires.ftc.teamcode.teamcode;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
//import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//@TeleOp
//public class VisionClass extends LinearOpMode{
//    public void runOpMode() {
//        Vision vision = new Vision(hardwareMap);
//    }
//}
//class Vision{
//    public AprilTagProcessor aprilTag;
//    public List<AprilTagDetection> currentDetections;
//    public VisionPortal visionPortal;
//    public VisionPortal.Builder builder;
//    public AprilTagProcessor tagProcessor;
//    private static final boolean USE_WEBCAM = true;
//    Vision(HardwareMap hardwareMap){
//        AprilTagMetadata myAprilTagMetadata;
//        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
//        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
//        AprilTagLibrary myAprilTagLibrary;
//        AprilTagProcessor myAprilTagProcessor;
//        myAprilTagMetadata = new AprilTagMetadata(20, "please work i beg", 6.5, DistanceUnit.INCH);
//        tagProcessor = new AprilTagProcessor.Builder()
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .build();
//        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
//        myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);
//        myAprilTagLibrary = myAprilTagLibraryBuilder.build();
//        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
//        aprilTag = myAprilTagProcessorBuilder.build();
//        builder = new VisionPortal.Builder();
//        if (USE_WEBCAM){
//            builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//        builder.addProcessor(aprilTag);
//        visionPortal = builder.build();        List<AprilTagDetection> detections = telemetryAprilTag();
//
//
//        visionPortal.close();
//    }
//
//    public String obelisk(){
//        List<AprilTagDetection> detections = telemetryAprilTag();
//        for (AprilTagDetection detection : tagProcessor.getDetections()) {
//            if (detection.id == 21) {
//                return "green, purple, purple";
//            } else if (detection.id == 22) {
//                return "purple, green, purple";
//            } else if (detection.id == 23) {
//                return "purple, purple, green";
//            } else {
//                return "no pattern";
//            }
//        }
//        return "no pattern";
//
//    }
//    public double distance(){
//        for (AprilTagDetection detection : tagProcessor.getDetections()) {
//            return detection.robotPose.getPosition().y;
//        }
//        return 0;
//
//    }
//
//private List<AprilTagDetection> telemetryAprilTag(){
//    currentDetections = aprilTag.getDetections();
//    return currentDetections;
//}   // end method telemetryAprilTag()
//
//static double clamp(double value, double min, double max) {
//    return Math.min(Math.max(min, value), max);
//} // end class
//public void printTelemetryVision(Telemetry telemetry){
//    telemetry.addData("# AprilTags Detected", currentDetections.size());
//}
//}