package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled//@Autonomous (name = "NearAutoToBeUsed")

public class ThreeSampleAutoPush extends MecanumDrivetrain{
  final double PI = Math.PI;
  
  //put the definitions for the motors and stuff here:
  //eg. private Motor leftMotor;
  private Servo rightClaw;
  private Servo leftClaw;
  
  private MotorController slideController = new MotorController(0.02, 0, 0.0003); // -4200
  private double slideTargetPosition = 0;
  
  private boolean clawIsOpen;
  private boolean slideIsUp;
  
  private CRServo outakeServo;
  
  private DcMotor armMotor;
  private MotorController armMotorController = new MotorController(0.04, 0, 0.0006);
  
  private DcMotor slideMotor;
  private DcMotor hangMotor;
  
  private RotationController rotationController;
  
  double armMotorTargetPosition = 0;
  byte armPosition = 4;
  
  private double motorUnitsToDegrees(double motorUnits) {
    return (0.073 * motorUnits) + 114.309;
  }
  
  private double degreesToMotorUnits(double degrees) {
    return (13.7 * degrees) - 1566;
  }
  
  public void turn(double degrees) {

    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    double radians = -degrees * (PI / 180);
    
    rotationController.resetIntegralSum();
    while (opModeIsActive()) {
      double[] powers = rotationController.updateMotor(getRobotHeading(), radians);
      
      if (Arrays.equals(powers, rotationController.DONE)) {
        break;
      }
      
      double rotationPower = powers[0] + powers[1] + powers[2];
      

      setDrivetrainPowers(0, 0, -rotationPower);
      
      telemetry.addData("p", powers[0]);
      telemetry.addData("i", powers[1]);
      telemetry.addData("d", powers[2]);
      
      telemetry.addData("turning...", rotationPower);
      telemetry.addData("robot current rotation", getRobotHeading());
      telemetry.addData("robot desired rotation", radians);
      telemetry.update();
    }
    setRunToPosition();
  }
  
public void move(double x, double y, int ticks, double speed) {
    // find the distance the motors have to travel
    double frontLeftDistance = y + x;
    double frontRightDistance = y - x;
    double backLeftDistance = y - x;
    double backRightDistance = y + x;
    
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    frontLeft.setTargetPosition( (int) backRightDistance * ticks);
    frontRight.setTargetPosition( (int) backLeftDistance * ticks);
    backLeft.setTargetPosition( (int) frontRightDistance * ticks);
    backRight.setTargetPosition( (int) frontLeftDistance * ticks);
    
    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    frontLeft.setPower(speed);
    frontRight.setPower(speed);
    backLeft.setPower(speed);
    backRight.setPower(speed);
    
    while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
      telemetry.addLine("moving");
      updateNonDrivetrain();
    }
}


  
  public void breakMotors() {
    double[] motorPositions = {frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition()};
    sleep(100);
    double[] newMotorPositions = {frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition()};
    
    frontLeft.setPower(-3 * (newMotorPositions[0] - motorPositions[0]));
    frontRight.setPower(-3 * (newMotorPositions[1] - motorPositions[1]));
    backLeft.setPower(-3 * (newMotorPositions[2] - motorPositions[2]));
    backRight.setPower(-3 * (newMotorPositions[3] - motorPositions[3]));
    sleep(100);
    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);
  }
  
  public void updateSlide() {
    if (slideIsUp) {
      slideTargetPosition = -4400;
    } else {
      slideTargetPosition = -10;
    }
    
    double slideMotorPIDPower = slideController.updateMotor(slideMotor.getCurrentPosition(), slideTargetPosition);
    if (Math.abs(slideMotorPIDPower) < 0.1) {
      slideMotorPIDPower = 0;
    }
    if (slideMotorPIDPower > 0 && slideMotor.getCurrentPosition() > -10) {
      slideMotorPIDPower = 0;
    }
    slideMotor.setPower(slideMotorPIDPower);
  }
  
  public void openClaw() {
    clawIsOpen = true;
    updateClaw();
  }
  
  public void closeClaw() {
    clawIsOpen = false;
    updateClaw();
  }
  
  public void updateClaw() {
    if (clawIsOpen) {
      rightClaw.setPosition(0.2);//rightClaw.setPosition(0.233);
      leftClaw.setPosition(0.496);//leftClaw.setPosition(0.426);

    } else {
      rightClaw.setPosition(0.4);
      leftClaw.setPosition(0.243);
    }
  }

  public void updateNonDrivetrain() { //runs in the busy loops in the drivetrain commands (move and turn)
    telemetry.addData("robot current rotation", getRobotHeading());

    if (armPosition == 1) {
      armMotorTargetPosition = degreesToMotorUnits(7);
      
    } else if (armPosition == 2) {
      armMotorTargetPosition = degreesToMotorUnits(20);
      
    } else if (armPosition == 3) {
      armMotorTargetPosition = degreesToMotorUnits(75);
      
    } else if (armPosition == 4) {
      armMotorTargetPosition = degreesToMotorUnits(115);
      
    }
    
    double armMotorPIDPower = armMotorController.updateMotor(armMotor.getCurrentPosition(), armMotorTargetPosition);
    
    if (Math.abs(armMotorPIDPower) < 0.1) {
      armMotorPIDPower = 0;
      telemetry.addLine("arm done");
    } 
    
    armMotor.setPower(armMotorPIDPower * 0.4);
    
    updateSlide();
    
    telemetry.update();
  }
  
  public void updateSleep(double seconds) {
    ElapsedTime timer = new ElapsedTime();
    while (timer.seconds() < seconds && opModeIsActive()) {
      updateNonDrivetrain();
    }
  }
  
  public void runOutakeServo(double seconds) {
    ElapsedTime timer = new ElapsedTime();
    outakeServo.setPower(-1);
    while (timer.seconds() < seconds && opModeIsActive()) {
      updateNonDrivetrain();
    }
    outakeServo.setPower(0);
  }
  
  public void grabSample() {
    openClaw();
    armPosition = 1;
    
    updateSleep(2);
    
    closeClaw();
    
    updateSleep(0.5);
    
    armPosition = 4;
    
    updateSleep(1.5);
    
    openClaw();
    
    updateSleep(0.5);
    
    armPosition = 3;
    
    updateSleep(2);
  }
  
  public void depositSample() {
    runOutakeServo(2);
  }
  
  public void startToBucket() {

  }
  
  public void firstSample() {

  }
  
  //executed when this OpMode is selected from the Driver Station
  @Override
  public void runOpMode() { //will run once you press init
    rightClaw = hardwareMap.get(Servo.class, "rc");
    leftClaw = hardwareMap.get(Servo.class, "lc");
    
    outakeServo = hardwareMap.get(CRServo.class, "outakeServo");
    
    armMotor = hardwareMap.get(DcMotor.class, "arm");
    hangMotor = hardwareMap.get(DcMotor.class, "hang");
    slideMotor = hardwareMap.get(DcMotor.class, "slide");
    
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    //rotationController = new RotationController(3, 0, 0, 0.03, 0.2);
    rotationController = new RotationController(3, 0, 0.1, 0.03, 0.2);
    
    initDrivetrain();
    
    clawIsOpen = true;
    slideIsUp = false;
    
    setRunToPosition();
    
    waitForStart(); //the following code will run once you press the triangle play button thing
    
    //>>>>CODE WE ACTUALLY CARE ABOUT:<<<<
    closeClaw();
    // start to basket
    armPosition = 3;
    move(0, -1, 100, 0.5);
    slideIsUp = true;
    
    move(1, 0, 950, 0.3);
    
    armPosition = 4;
    
    updateSleep(1);
  
    runOutakeServo(1.6);
    
    hangMotor.setPower(-1);
    updateSleep(0.5);
    hangMotor.setPower(0);
    
    // first sample grab
    move(-1, -1, 500, 1);
 
    updateSleep(0.3);
    openClaw();
    armPosition = 3;
    slideIsUp = false;
    
    updateSleep(0.75);
    turn(180);
    
    move(1, 0, -150, 0.3);
    move(0, 1, 70, 0.2);
    updateSleep(0.5);
    //move(-1, 0, 50, 0.2);//jfioaskod
    //updateSleep(1);
    armPosition = 2;
    updateSleep(1.5);
    armPosition = 1;
    updateSleep(1.5);
    closeClaw();
    
    // go to bucket
    
    updateSleep(0.5);
    armPosition = 4;
    updateSleep(2.5);
    openClaw();
    updateSleep(0.5);
    
    turn(0);
    
    updateSleep(0.75);
    armPosition = 3;
    
    updateSleep(1);//move(0, 1, 300, 0.5);
    slideIsUp = true;
    move(1, 1, 400, 0.6);
    updateSleep(1.5);
    
    armPosition = 4;
    move(1, 1, 100, 0.3);
    updateSleep(1);
    runOutakeServo(1.475);
    
    //updateSleep(1);
    
    slideMotor.setPower(-0.5);
    sleep(100);
    slideMotor.setPower(0);
    move(1, 1, -1100, 1);
    armPosition = 3;
    slideIsUp = false;
    updateSleep(0.5);
    
    move(1, 0, -150, 1);
    updateSleep(0.1);
    
    move(0, -1, 1950, 1);
    updateSleep(0.5);
    
    // move(1, 0, 750, 1);
    // updateSleep(0.25);
    
    // move(0, 1, 1900, 1);
    // armPosition = 4;
    // updateSleep(0.1);
    
    // move(1, 1, -800, 1);
    // updateSleep(0.001);
    
    // move(0, -1, 1900, 1);
    // updateSleep(0.001);
    
    
    turn(90);
    updateSleep(0.001);
    
    move(0, -1, 1500, 0.35);
    updateSleep(0.001);
    
    sleep(400);
    
    
    
    
    
    // end of code we actually care about
  }
}

class RotationController1 { //this code is so assssssss
  
  final double PI = Math.PI;
  
  public static final double[] DONE = {100, 100, 100};
  
  double tolerance = 10; //how close the motor has to get to its target to stop
  double secondsToEnd = 0.07; //how long the motor has to be within the tolerance to stop

  ElapsedTime secondsWithinTolerance;

  double Kp;

  double Ki; //integral term
  double integralSum; //used for calculating the integral

  double Kd; //derivative term
  double lastError; //used for calculating the derivative
  
  ElapsedTime timer;
  
  private static double bound(double num, double lowerBound, double upperBound) {
      return Math.max(Math.min(num, upperBound), lowerBound);
    }
  
  RotationController1(double Kp, double Ki, double Kd, double tolerance, double secondsToEnd) {
    this.secondsWithinTolerance = new ElapsedTime();
    
    this.Kp = Kp;
    this.Ki = Ki;
    this.Kd = Kd;
  
    this.integralSum = 0;
    this.lastError = 0;
  
    this.timer = new ElapsedTime();
  
    this.tolerance = tolerance;
    this.secondsToEnd = secondsToEnd;
  }
  
  
  private double[] PIDController(double error) {
    //the sum of change over time
    integralSum += error * timer.seconds();
    
    integralSum = bound(integralSum, -0.2, 0.2);
  
    //the rate of change of the error
    double derivative = (error - lastError) / timer.seconds();
  
    lastError = error;
  
    timer.reset();
    double[] outputs = {(error * Kp), (integralSum * Ki), (derivative * Kd)};
    return outputs;
    //return bound((error * Kp) + (integralSum * Ki) + (derivative * Kd), -1, 1);
  }
  
  public double[] updateMotor(double currentPosition, double reference) {
    double actualError = reference - currentPosition;
    double leastError = actualError;
    
    double[] errors = {actualError - (4 * PI), actualError - (2 * PI), actualError + (2 * PI)};
    
    for (double error : errors) {
      if (Math.abs(leastError) > Math.abs(error)) {
        leastError = error;
      }
    }
    
    double[] output = PIDController(leastError);
    
    if (! (leastError < tolerance && leastError > -tolerance)) {
      secondsWithinTolerance.reset();
    }
    if (secondsWithinTolerance.seconds() >= secondsToEnd) {
      return DONE;
    }
    
    return output;
  }
  
  
  public void resetIntegralSum(){
    integralSum = 0;
  }
}










