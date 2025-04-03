package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled//@TeleOp(name = "HermelaTestCode (Blocks to Java)")
public class HermelaTestCode extends LinearOpMode {

  private DcMotor bl;
  private DcMotor fl;
  private DcMotor br;
  private DcMotor fr;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    bl = hardwareMap.get(DcMotor.class, "bl");
    fl = hardwareMap.get(DcMotor.class, "fl");
    br = hardwareMap.get(DcMotor.class, "br");
    fr = hardwareMap.get(DcMotor.class, "fr");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      bl.setDirection(DcMotor.Direction.REVERSE);
      fr.setDirection(DcMotor.Direction.REVERSE);
      br.setDirection(DcMotor.Direction.REVERSE);
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        if (gamepad1.dpad_up) {
          bl.setPower(1);
          br.setPower(1);
          fl.setPower(1);
          fr.setPower(1);
        }
        if (gamepad1.dpad_down) {
          bl.setPower(-1);
          br.setPower(-1);
          fl.setPower(-1);
          fr.setPower(-1);
        }
        if (gamepad1.dpad_right) {
          bl.setPower(1);
          br.setPower(-1);
          fl.setPower(1);
          fr.setPower(-1);
        }
        if (gamepad1.dpad_left) {
          bl.setPower(-1);
          br.setPower(1);
          fl.setPower(-1);
          fr.setPower(1);
        }
        if (gamepad1.left_bumper) {
          bl.setPower(1);
          br.setPower(-1);
          fl.setPower(-1);
          fr.setPower(1);
        }
        if (gamepad1.right_bumper) {
          bl.setPower(-1);
          br.setPower(1);
          fl.setPower(1);
          fr.setPower(-1);
        }
      }
    }
  }
}