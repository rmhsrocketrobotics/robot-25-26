package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Disabled//@TeleOp

public class TestTeleop extends LinearOpMode{

    @Override
        public void runOpMode() {
            DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
            DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
            DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
            DcMotor br = hardwareMap.get(DcMotor.class, "br");
            
            //fl.setDirection(DcMotorSimple.Direction.REVERSE);
            //bl.setDirection(DcMotorSimple.Direction.REVERSE);
            
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            waitForStart();
            
            while(opModeIsActive()) {
                double y = gamepad1.left_stick_x;
                double x = gamepad1.right_stick_x;
                double rx = gamepad1.right_stick_y;
                fr.setPower((y + x + rx) * 0.6);
                br.setPower((y - x + rx) * 0.6);
                fl.setPower((y + x - rx) * 0.6);
                bl.setPower((y - x - rx) * 0.6);
            }
            
        }
    }
