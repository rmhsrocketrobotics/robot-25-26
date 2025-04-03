package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled//@Autonomous(name="AutoBasketTest")

public class AutoBasket extends LinearOpMode {
  
  private DcMotor br;
  private DcMotor fr;
  private DcMotor arm;
  private DcMotor slide;
  private DcMotor bl;
  private DcMotor fl;
  private CRServo outakeServo;
  private MotorController armMotorController;
  private double armMotorTargetPosition;
  
  public void reset() {
    fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }
  
  public void run() {
    fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while(fl.isBusy()) {
      arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
  }
    
  public void setTicks(int frontL, int backL, int frontR, int backR) {
    fl.setTargetPosition(frontL);
    fr.setTargetPosition(backL);
    bl.setTargetPosition(frontR);
    br.setTargetPosition(backR);
  }
  public void resetArm() {
    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }
  public void runArm1() {
    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while(arm.isBusy());
  }
  
  private void runArm(int ticks) {
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm.setTargetPosition(ticks);
    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //arm.setPower(0.5);
    // while (opModeIsActive() && arm.isBusy()) {
    //   idle();
    // }
    while (arm.isBusy());
  }
  
  public void runOpMode() {
     br = hardwareMap.get(DcMotor.class, "br");
    fr = hardwareMap.get(DcMotor.class, "fr");
    slide = hardwareMap.get(DcMotor.class, "slide");
    bl = hardwareMap.get(DcMotor.class, "bl");
    fl = hardwareMap.get(DcMotor.class, "fl");
    outakeServo = hardwareMap.get(CRServo.class, "outakeServo");
    arm = hardwareMap.get(DcMotor.class, "arm");

    br.setDirection(DcMotor.Direction.REVERSE);
    fr.setDirection(DcMotor.Direction.REVERSE);
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    fl.setTargetPosition(0);
    fr.setTargetPosition(0);
    bl.setTargetPosition(0);
    br.setTargetPosition(0);
    arm.setTargetPosition(0);

    fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
    fl.setPower(0.3);
    fr.setPower(0.3);
    bl.setPower(0.3);
    br.setPower(0.3);
    arm.setPower(0.4);
    
    waitForStart();
    
    // reset();
    // setTicks(-500, -500, -500, -500);
    // run();
    
    runArm(-17);
    sleep(500);
    slide.setPower(-0.6);
    sleep(1000);
    slide.setPower(0);
    while(opModeIsActive()) {
      arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      telemetry.addData("armPosition", arm.getCurrentPosition());
      telemetry.update();
    }

  }


}