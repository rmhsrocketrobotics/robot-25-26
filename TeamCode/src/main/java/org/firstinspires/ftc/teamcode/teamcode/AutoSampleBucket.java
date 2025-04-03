import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled//@Autonomous(name = "AutoSampleBucket (Blocks to Java)")
public class AutoSampleBucket extends LinearOpMode {

  private DcMotor br;
  private DcMotor fr;
  private DcMotor arm;
  private DcMotor slide;
  private DcMotor bl;
  private DcMotor fl;
  private CRServo outakeServo;
  
  private double convertToDegrees(double motorUnits) {
    return (0.647 * motorUnits) - 1.941;
  }

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
    arm = hardwareMap.get(DcMotor.class, "arm");
    slide = hardwareMap.get(DcMotor.class, "slide");
    bl = hardwareMap.get(DcMotor.class, "bl");
    fl = hardwareMap.get(DcMotor.class, "fl");
    outakeServo = hardwareMap.get(CRServo.class, "outakeServo");

    br.setDirection(DcMotor.Direction.REVERSE);
    fr.setDirection(DcMotor.Direction.REVERSE);
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm.setTargetPosition(0);
    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm.setPower(0.3);
    
    waitForStart();
    
    DriveWithEncoder(0.4, -500, -500, -500, -500);
    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm.setTargetPosition(-40);
    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while(arm.isBusy());
    
    runSlide(0.7, 3.9);
    
    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm.setTargetPosition(1);
    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while(arm.isBusy());
    
    DriveWithEncoder(0.2, -1200, 1200, 1200, -1200);
    DriveWithEncoder(0.2, 700, 0, 0, -700);
    runSlide(0.7, 3.9);
    runOuttake(5);
    runSlide(-0.7, 2.9);
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
  private void runOuttake(int secRun) {
    outakeServo.setPower(-1);
    sleep(1000 * secRun);
    outakeServo.setPower(0);
  }
}