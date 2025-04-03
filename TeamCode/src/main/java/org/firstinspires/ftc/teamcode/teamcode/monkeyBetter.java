import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled//@TeleOp(name = "Hehehe Hah")
public class monkeyBetter extends LinearOpMode {

  private Servo wrist;
  private Servo plane;
  private Servo claw;
  private DcMotor rightMotor;
  private DcMotor leftMotor;
  private DcMotor elbow;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    wrist = hardwareMap.get(Servo.class, "wrist");
    plane = hardwareMap.get(Servo.class, "plane");
    claw = hardwareMap.get(Servo.class, "claw");
    rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
    leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
    elbow = hardwareMap.get(DcMotor.class, "elbow");

    // Put initialization blocks here.
    wrist.setPosition(0.5);
    plane.setPosition(0.5);
    claw.setPosition(0.5);
    rightMotor.setDirection(DcMotor.Direction.REVERSE);
    boolean sprint = false;
    
    waitForStart();
    while (opModeIsActive()) {
      // Put loop blocks here.
      if (gamepad1.b) {
        plane.setPosition(1);
      }
      
      if(gamepad2.b){
        sprint = !sprint;
      }
      
      if(sprint){
        rightMotor.setPower(gamepad2.right_stick_y);
      leftMotor.setPower(gamepad2.left_stick_y);
      }{
      rightMotor.setPower(0.7 * gamepad2.right_stick_y);
      leftMotor.setPower(0.7 * gamepad2.left_stick_y);
      }
      elbow.setPower(0.7 * gamepad1.right_stick_y);
      if (gamepad1.right_bumper) {
        wrist.setPosition(0.9);
      } else if (gamepad1.left_bumper) {
        wrist.setPosition(1);
      }
      if (gamepad1.dpad_down) {
        claw.setPosition(0.4);
      } else if (gamepad1.dpad_right) {
        claw.setPosition(0.55);
      }
      telemetry.addData("Hand position", claw.getPosition());
      telemetry.addData("Wrist position", wrist.getPosition());
      telemetry.addData("Sprint: ", sprint);
      telemetry.update();
    }
    if (opModeIsActive()) {
      // Put run blocks here.
    }
  }
}