package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled//@Autonomous()
public class SampleAndAscentAuto extends LinearOpMode {

  private DcMotor br;
  private DcMotor fr;
  private CRServo outakeServo;
  private DcMotor slide;
  private DcMotor arm;
  private DcMotor bl;
  private DcMotor fl;
  private DcMotor hang;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    br = hardwareMap.get(DcMotor.class, "br");
    fr = hardwareMap.get(DcMotor.class, "fr");
    outakeServo = hardwareMap.get(CRServo.class, "outakeServo");
    slide = hardwareMap.get(DcMotor.class, "slide");
    arm = hardwareMap.get(DcMotor.class, "arm");
    bl = hardwareMap.get(DcMotor.class, "bl");
    fl = hardwareMap.get(DcMotor.class, "fl");
    
    hang = hardwareMap.get(DcMotor.class, "hang");
    
    hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    br.setDirection(DcMotor.Direction.REVERSE);
    fr.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();
    
    //sample in bucket
    DriveWithEncoder(0.2, -150, -150, -150, -150);
    runArm(0.4, 0.7);
    runSlide(0.7, 3.25);
    DriveWithEncoder(0.2, -1500, 1500, 1500, -1500);
    RightPivot(300, 300);
    DriveWithEncoder(0.3, 100, 100, 100, 100);
    runSlide(0.7, 0.35);
    runOuttake(5);
    runSlide(0.7, 0.1);
    DriveWithEncoder(0.2, -300, -300, -300, -300);
    runSlide(-0.7, 2.9);
    DriveWithEncoder(0.2, -200, -200, -400, -400);
    
    //level one ascent
    hang.setPower(-0.5);
    sleep(800);
    hang.setPower(0);
    
    DriveWithEncoder(0.2, -200, -200, -400, -400);
    
    // DriveWithEncoder(0.2, -150, -250, -350, -450);
    
    // DriveWithEncoder(0.3, -550, -650, -750, -850);
    // DriveWithEncoder(0.2, -500, -500, -600, -600);
    // DriveWithEncoder(0.2, -700, -700, -800, -800);
  }

  /**
   * Describe this function...
   */
  private void runOuttake(int secRun) {
    outakeServo.setPower(-1);
    sleep(1000 * secRun);
    outakeServo.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void runSlide(double slideSpeed, double SecRise) {
    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide.setPower(-1 * slideSpeed);
    sleep((long) (1000 * SecRise));
    while (opModeIsActive() && slide.isBusy()) {
      idle();
    }
    slide.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void runArm(double slideSpeed, double SecRise) {
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    arm.setPower(-1 * slideSpeed);
    sleep((long) (1000 * SecRise));
    while (opModeIsActive() && arm.isBusy()) {
      idle();
    }
    arm.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void DriveWithEncoder(double speed, int frontRight, int backRight, int frontLeft, int backLeft) {
    br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    br.setTargetPosition(backRight);
    bl.setTargetPosition(backLeft);
    fr.setTargetPosition(frontRight);
    fl.setTargetPosition(frontLeft);
    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fr.setPower(speed);
    fl.setPower(speed);
    bl.setPower(speed);
    br.setPower(speed);
    while (opModeIsActive() && fr.isBusy() && fl.isBusy() && br.isBusy() && bl.isBusy()) {
      idle();
    }
  }

  /**
   * Describe this function...
   */
  private void RightPivot(int blPivot, int frPivot) {
    bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bl.setTargetPosition(-1 * blPivot);
    fr.setTargetPosition(frPivot);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setPower(0.4);
    fr.setPower(0.4);
    while (opModeIsActive() && bl.isBusy() && fr.isBusy()) {
      idle();
    }
  }
}