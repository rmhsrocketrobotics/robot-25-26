package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled//@TeleOp(name = "BrynnOpMode (Blocks to Java)")
public class BrynnOpMode extends LinearOpMode {

  private Servo wrist;
  private Servo plane;
  private Servo claw;
  private DcMotor backrightMotor;
  private DcMotor backleftMotor;
  private DcMotor elbow;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    wrist = hardwareMap.get(Servo.class, "wrist");
    plane = hardwareMap.get(Servo.class, "plane");
    claw = hardwareMap.get(Servo.class, "claw");
    backrightMotor = hardwareMap.get(DcMotor.class, "back rightMotor");
    backleftMotor = hardwareMap.get(DcMotor.class, "back leftMotor");
    elbow = hardwareMap.get(DcMotor.class, "elbow");

    wrist.setPosition(0.5);
    plane.setPosition(0.52);
    claw.setPosition(0.4);
    backrightMotor.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();
    while (opModeIsActive()) {
      // Put loop blocks here.
      if (gamepad1.y) {
        plane.setPosition(1);
      }
      backrightMotor.setPower(1 * gamepad2.right_stick_y);
      backleftMotor.setPower(1 * gamepad2.left_stick_y);
      elbow.setPower(0.7 * gamepad1.right_stick_y);
      if (gamepad1.right_bumper) {
        wrist.setPosition(0.8);
      } else if (gamepad1.left_bumper) {
        wrist.setPosition(1);
      } else if (gamepad1.a) {
        wrist.setPosition(0.35);
      }
      if (gamepad1.dpad_down) {
        claw.setPosition(0.32);
      } else if (gamepad1.dpad_right) {
        claw.setPosition(0.47);
      }
      telemetry.addData("Hand position", claw.getPosition());
      telemetry.addData("Wrist position", wrist.getPosition());
      telemetry.update();
    }
    if (opModeIsActive()) {
      // Put run blocks here.
    }
    // Put initialization blocks here.
  }
}
