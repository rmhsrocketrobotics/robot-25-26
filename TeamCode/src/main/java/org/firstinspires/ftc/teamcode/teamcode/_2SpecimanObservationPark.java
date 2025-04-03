package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled//@Autonomous(name = "_2SpecimanObservationPark (Blocks to Java)")
public class _2SpecimanObservationPark extends LinearOpMode {

  private DcMotor br;
  private DcMotor fr;
  private DcMotor hang;
  private DcMotor bl;
  private DcMotor fl;
  private DcMotor arm;

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
    hang = hardwareMap.get(DcMotor.class, "hang");
    bl = hardwareMap.get(DcMotor.class, "bl");
    fl = hardwareMap.get(DcMotor.class, "fl");
    arm = hardwareMap.get(DcMotor.class, "arm");

    br.setDirection(DcMotor.Direction.REVERSE);
    fr.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();
    hang2(-0.8, 1250);
    DriveWithEncoder(0.4, -1200, -1200, -1200, -1200);
    hang2(1, 1000);
    DriveWithEncoder(0.35, 600, 600, 600, 600);
    DriveWithEncoder(0.4, -2000, -2000, 2000, 2000);
    DriveWithEncoder(0.4, -2100, 2100, 2100, -2100);
    DriveWithEncoder(0.5, -1100, -1100, -1100, -1100);
    hang2(-0.5, 900);
    DriveWithEncoder(0.5, 900, 900, 900, 900);
    hang2(0.4, 900);
    DriveWithEncoder(0.4, 2000, -2000, -2000, 2000);
    DriveWithEncoder(0.4, -2000, -2000, 2000, 2000);
    hang2(-0.8, 1250);
    DriveWithEncoder(0.4, -350, -350, -350, -350);
    hang2(1, 1000);
    DriveWithEncoder(0.35, 600, 600, 600, 600);
  }

  /**
   * Describe this function...
   */
  private void hang2(double hangpower, int msec) {
    hang.setPower(hangpower);
    sleep(msec);
    hang.setPower(0);
    hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  /**
   * Describe this function...
   */
  private void DriveWithEncoder(double speed, int frontRight, int backRight, int frontLeft, int backLeft) {
    br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    br.setTargetPosition(backRight);
    bl.setTargetPosition(backLeft);
    fr.setTargetPosition(frontRight);
    fl.setTargetPosition(frontLeft);
    br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fl.setPower(speed);
    fr.setPower(speed);
    br.setPower(speed);
    bl.setPower(speed);
    while (opModeIsActive() && br.isBusy() && bl.isBusy() && fl.isBusy() && fr.isBusy()) {
      idle();
    }
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
}
