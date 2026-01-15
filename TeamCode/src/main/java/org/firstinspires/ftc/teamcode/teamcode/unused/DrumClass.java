//package org.firstinspires.ftc.teamcode.teamcode;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import android.app.Activity;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import android.graphics.Color;
//import android.view.View;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//
//
//@Disabled//@TeleOp
//public class DrumClass extends LinearOpMode {
//
//    public void runOpMode(){
//        Drum drum = new Drum(hardwareMap, telemetry);
//    }
//}
//class Drum {
//    public String[] ballStates = {"none", "none", "none"};
//    public ElapsedTime switchTimer = new ElapsedTime();
//    public ServoImplEx drumServo;
//
//    public NormalizedColorSensor colorSensor;
//    public int drumPos;
//    public NormalizedRGBA colors;
//    public float[] hsvValues;
//    Drum(HardwareMap hardwareMap, Telemetry telemetry) {
//
//    }
//        public boolean detectColor(){
//            if (switchTimer.seconds() < 1) {
//                return false;
//            }
//            colors = colorSensor.getNormalizedColors();
//            hsvValues = new float[3];
//            Color.colorToHSV(colors.toColor(), hsvValues);
//
//            String color = "unknown";
//
//
//            if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 3 && ballStates[drumPos] == "none"){
//                if (hsvValues[0] > 200) {
//                    color = "purple";
//                } else if (hsvValues[0] < 180) {
//                    color = "green";
//                }
//                ballStates[drumPos] = color;
//                //telemetry.addData("ball detected! color", color);
//                return true;
//            }
//
//            return false;
//        }
//        public void intakeDrum(int ballSlot) {
//            double[] positions = {0, 0.3826, 0.7831};
//            drumServo.setPosition(positions[ballSlot]);
//        }
//        public void outtakeDrum(int ballSlot) {
//            double[] positions = {0.5745, 0.95, 0.1823};
//            drumServo.setPosition(positions[ballSlot]);
//        }
//    public void printTelemetryDrum(Telemetry telemetry){
//        telemetry.addLine()
//                .addData("Red", "%.3f", colors.red)
//                .addData("Green", "%.3f", colors.green)
//                .addData("Blue", "%.3f", colors.blue);
//        telemetry.addLine()
//                .addData("Hue", "%.3f", hsvValues[0]);
//        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
//
//        telemetry.addData("ballStates0", ballStates[0]);
//        telemetry.addData("ballStates1", ballStates[1]);
//        telemetry.addData("ballStates2", ballStates[2]);
//    }
//
//}
