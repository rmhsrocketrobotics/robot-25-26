package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;

@TeleOp(group = "!main")

public class MainTeleop extends LinearOpMode{
    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Outtake outtake = new Outtake(hardwareMap, true);

        DcMotor flick = hardwareMap.get(DcMotor.class, "flick");
        CRServo flickServo = hardwareMap.get(CRServo.class, "flickServo");

        Gamepad gamepad1Last = new Gamepad();
        Gamepad gamepad2Last = new Gamepad();

        //random ahh imu stuff dw
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        //telemetry.setMsTransmissionInterval(250); //default 250
        //telemetry.setNumDecimalPlaces(0, 5);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y && !gamepad1Last.y) {
                telemetry.speak("test test 1 2 3");
            }
            gamepad1Last.copy(gamepad1);

            drivetrain.setDrivetrainPower(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            double desiredTPS = -gamepad2.left_stick_y * 1000;
            outtake.setOuttakeVelocityTPS(desiredTPS);
            telemetry.addData("desiredTPS", desiredTPS);

            //intake.setPower(-gamepad2.right_stick_y);
            flick.setPower(gamepad2.left_trigger);
            flickServo.setPower(gamepad2.right_stick_y);

            drivetrain.printTelemetry(telemetry);
            outtake.printTelemetry(telemetry);
            telemetry.update();
        }
    }
}

class Drivetrain {
    public DcMotor flMotor;
    public DcMotor frMotor;
    public DcMotor blMotor;
    public DcMotor brMotor;
    Drivetrain(HardwareMap hardwareMap) {
        flMotor = hardwareMap.get(DcMotor.class, "fl");
        frMotor = hardwareMap.get(DcMotor.class, "fr");
        blMotor = hardwareMap.get(DcMotor.class, "bl");
        brMotor = hardwareMap.get(DcMotor.class, "br");

        //change the direction of drivetrain motors so they're all facing the same way
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        //make the motors brake
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset encoder values
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //run w/o velocity pid
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivetrainPower(double yPower, double xPower, double rPower) {
        //equations to move the robot
        flMotor.setPower(yPower + xPower + rPower);
        frMotor.setPower(yPower - xPower - rPower);
        blMotor.setPower(yPower - xPower + rPower);
        brMotor.setPower(yPower + xPower - rPower);
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("fldrivetrain is ", flMotor.getCurrentPosition());
        telemetry.addData("frdrivetrain is ", frMotor.getCurrentPosition());
        telemetry.addData("bldrivetrain is ", blMotor.getCurrentPosition());
        telemetry.addData("brdrivetrain is ", brMotor.getCurrentPosition());
    }
}

class Outtake {
    DcMotorEx outtake1;
    DcMotorEx outtake2;
    PIDFCoefficients MOTOR_VELO_PID;
    VoltageSensor batteryVoltageSensor;
    Outtake(HardwareMap hardwareMap, boolean useVeloPID) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        if (useVeloPID) {
            MOTOR_VELO_PID = new PIDFCoefficients(25, 0, 0, 19);

            MotorConfigurationType motorConfigurationType = outtake1.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            outtake1.setMotorType(motorConfigurationType);
            outtake2.setMotorType(motorConfigurationType);

            outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
            setPIDFCoefficients(outtake1, MOTOR_VELO_PID);
            setPIDFCoefficients(outtake2, MOTOR_VELO_PID);
        }
    }

    public void setOuttakePower(double power) {
        outtake1.setPower(-power);
        outtake2.setPower(-power);
    }

    public void setOuttakeVelocityTPS(double ticksPerSecond) {
        outtake1.setVelocity(ticksPerSecond);
        outtake2.setVelocity(ticksPerSecond);
    }

    public void printTelemetry(Telemetry telemetry) {
        double outtakeVelocity = outtake1.getVelocity();
        telemetry.addData("outtake ticks/sec", outtakeVelocity);
        telemetry.addData("outtake rev/min", (outtakeVelocity * 60) / 28);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
}

//class Vision {
//    public void aprilTags(){
//
//        List<AprilTagDetection> detections = telemetryAprilTag();
//        if (gamepad1.dpad_down){
//            visionPortal.stopStreaming();
//        } else if (gamepad1.dpad_up){
//            visionPortal.resumeStreaming();
//        }
//        sleep(20);
//    }
//
//        visionPortal.close();
//    }
//    public String obelisk(){
//        List<AprilTagDetection> detections = telemetryAprilTag();
//        for (AprilTagDetection detection : tagProcessor.getDetections()) {
//            if (detection.id = 21) {
//                return "green, purple, purple";
//            } else if (detection.id = 22) {
//                return "purple, green, purple";
//            } else if (detection.id = 23) {
//                return "purple, purple, green";
//            } else {
//                return "no pattern";
//            }
//        }
//    }
//    public double distance(){
//        for (AprilTagDetection detection : tagProcessor.getDetections()) {
//            return detection.robotPose.getPosition().y;
//        }
//    }
//}
//
//private void initAprilTag(){
//    AprilTagMetadata myAprilTagMetadata;
//    AprilTagLibrary.Builder myAprilTagLibraryBuilder;
//    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
//    AprilTagLibrary myAprilTagLibrary;
//    AprilTagProcessor myAprilTagProcessor;
//
//    public myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
//    myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);
//    myAprilTagLibrary = myAprilTagLibraryBuilder.build();
//    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//    myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
//    public AprilTagProcessor aprilTag = myAprilTagProcessorBuilder.build();
//    public VisionPortal.Builder builder = new VisionPortal.Builder();
//    if (USE_WEBCAM){
//        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
//    } else {
//        builder.setCamera(BuiltinCameraDirection.BACK);
//    }
//    builder.addProcessor(aprilTag);
//    public VisionPortal visionPortal = builder.build();
//}
//private List<AprilTagDetection> telemetryAprilTag(){
//    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//    return currentDetections.size();
//    return currentDetections;
//}   // end method telemetryAprilTag()
//
//static double clamp(double value, double min, double max) {
//    return Math.min(Math.max(min, value), max);
//} // end class

