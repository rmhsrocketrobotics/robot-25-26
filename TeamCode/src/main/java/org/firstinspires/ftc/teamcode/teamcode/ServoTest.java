package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled//@TeleOp
public class ServoTest extends LinearOpMode {
  //put the definitions for the motors and stuff here:
  //eg. private Motor leftMotor;
  private CRServo intakeServo;
  private CRServo outakeServo;
  
  // controlled using start + A
  private void controller1() {
    intakeServo.setPower(gamepad1.right_stick_x);
  
    outakeServo.setPower(gamepad1.left_stick_x);
  }
  
  // controlled using start + B (wheel movement + plane)
  private void controller2() {
    //code for controller 2 (nah REALLY?)
  }
  
  //executed when this OpMode is selected from the Driver Station
  @Override
  public void runOpMode() { //will run once you press init
    intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
    outakeServo = hardwareMap.get(CRServo.class, "outakeServo");
    
    double intake = gamepad1.right_stick_x;
    double outake = gamepad1.left_stick_x;
    
    waitForStart(); //the following code will run once you press the triangle thing
    
    
    while (opModeIsActive()) { //runs in a loop while op mode is running
      controller1();
      intake = gamepad1.right_stick_x;
      outake = gamepad1.left_stick_x;
      //controller2();
      
      telemetry.addData("intakeServo", intake);
      telemetry.addData("outakeServo", outake);
      telemetry.update();
    }
  }
}
