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

@Disabled//@Autonomous

public class New3Sample extends MecanumDrivetrain2{
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
  private MotorController armMotorController = new MotorController(0.08, 0, 0.0008);
  
  private DcMotor slideMotor;
  private DcMotor hangMotor1;
  private DcMotor hangMotor2;
  
  private RotationController rotationController;
  
  double armMotorTargetPosition = 0;
  byte armPosition = 4;
  
  private double motorUnitsToDegrees(double motorUnits) {
    return (0.073 * motorUnits) + 114.309;
  }
  
  private double degreesToMotorUnits(double degrees) {
    return (13.7 * degrees) - 1566;
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
      rightClaw.setPosition(0.18);//rightClaw.setPosition(0.2);
      leftClaw.setPosition(0.51);//leftClaw.setPosition(0.496);

    } else {
      rightClaw.setPosition(0.4);
      leftClaw.setPosition(0.243);
    }
  }

  public void updateNonDrivetrain() { //runs in the busy loops in the drivetrain commands (move and turn)
    telemetry.addData("arm degreese", motorUnitsToDegrees(armMotor.getCurrentPosition()));
      
    if (armPosition == 1) {
      armMotorTargetPosition = degreesToMotorUnits(5);
      
    } else if (armPosition == 2) {
      armMotorTargetPosition = degreesToMotorUnits(25);
      
    } else if (armPosition == 3) {
      armMotorTargetPosition = degreesToMotorUnits(75);
      
    } else if (armPosition == 4) {
      armMotorTargetPosition = degreesToMotorUnits(105);
      
    }
    
    double armMotorPIDPower = armMotorController.updateMotor(armMotor.getCurrentPosition(), armMotorTargetPosition);
    
    if (Math.abs(armMotorPIDPower) < 0.1) {
      armMotorPIDPower = 0;
      telemetry.addLine("arm done");
    } 
    
    armMotor.setPower(armMotorPIDPower * 0.5);
    
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
    outakeServo.setPower(-0.25);
    while (timer.seconds() < seconds && opModeIsActive()) {
      updateNonDrivetrain();
    }
    outakeServo.setPower(0);
    updateSleep(0.25);
  }
    @Override
    public void move(double xTicks, double yTicks) {
        move(xTicks, yTicks, 0.5);
    }
    
    //treat this like a graph, positive x is right, positive y is up (forward)
    @Override
    public void move(double xTicks, double yTicks, double speed) { // lets you set a position for the motors and let the pid do the work
        xTicks = -xTicks;
        yTicks = -yTicks;
        
        double startXPosition = getXPosition();
        double startYPosition = getYPosition();

        double currentXPosition = getXPosition() - startXPosition;
        double currentYPosition = getYPosition() - startYPosition;

        double targetXPosition = xTicks;
        double targetYPosition = yTicks;

        while (opModeIsActive() && (!xController.completed || !yController.completed)) {
            currentXPosition = getXPosition() - startXPosition;
            currentYPosition = getYPosition() - startYPosition;

            double xPower = xController.update(currentXPosition, targetXPosition);
            double yPower = yController.update(currentYPosition, targetYPosition);

            setDrivetrainPowers(xPower * speed, yPower * speed, 0);
            
            telemetry.addData("start y", startYPosition);
            telemetry.addData("current y", currentYPosition);
            telemetry.addData("target y", targetYPosition);
            telemetry.update();
            
            updateNonDrivetrain();
        }
        setDrivetrainPowers(0, 0, 0);
        xController.reset();
        yController.reset();
    }
    
    @Override
    public void turn(double degrees) {
        turn(degrees, 0.75);
    }
    
    @Override
    public void turn(double degrees, double speed) { // lets you set a position for the motors and let the pid do the work
        degrees = -degrees;
        
        double startDegrees = getRotation();

        double currentDegrees = getRotation() - startDegrees;

        double targetDegrees = degrees * 11; //insert conversion factor here

        while (opModeIsActive() && (!rController.completed)) {
            currentDegrees = getRotation() - startDegrees;

            double rPower = rController.update(currentDegrees, targetDegrees);

            setDrivetrainPowers(0, 0, rPower * speed);
            
            telemetry.addData("rController.completed", rController.completed);
            telemetry.addData("start", startDegrees);
            telemetry.addData("current", currentDegrees);
            telemetry.addData("target", targetDegrees);
            telemetry.update();
            
            updateNonDrivetrain();
        }
        setDrivetrainPowers(0, 0, 0);
        rController.reset();
    }
  
  
  //executed when this OpMode is selected from the Driver Station
  @Override
  public void runOpMode() { //will run once you press init
    rightClaw = hardwareMap.get(Servo.class, "rc");
    leftClaw = hardwareMap.get(Servo.class, "lc");
    
    outakeServo = hardwareMap.get(CRServo.class, "outakeServo");
    
    armMotor = hardwareMap.get(DcMotor.class, "arm");
    hangMotor1 = hardwareMap.get(DcMotor.class, "hangMotor1");
    hangMotor2 = hardwareMap.get(DcMotor.class, "hangMotor2");
    slideMotor = hardwareMap.get(DcMotor.class, "slide");
    
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    hangMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    hangMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    //rotationController = new RotationController(3, 0, 0, 0.03, 0.2);
    
    initDrivetrain();
    
    clawIsOpen = true;
    slideIsUp = false;
    
    waitForStart(); //the following code will run once you press the triangle play button thing
    
    openClaw();
    
    //>>>>CODE WE ACTUALLY CARE ABOUT:<<<<
    // start to basket:
    armPosition = 3;
    move(0, -100);
    slideIsUp = true;
    
    move(500, 0);
    updateSleep(1);
    armPosition = 4;
    move(550, 50);
    //move(0, 40);
    
    runOutakeServo(0.6);
    
    // basket to 1st sample:
    
    armPosition = 3;
    move(-750, -375);
    slideIsUp = false;
    
    turn(165);
    //updateSleep(0.5);
    
    armPosition = 2;
    //move(0, -100);
    updateSleep(0.75);
    armPosition = 1;
    updateSleep(0.5);
    closeClaw();
    updateSleep(0.5);
    
    //1st sample grabbed, putting it bucket:
    armPosition = 4;
    updateSleep(1.5);
    openClaw();
    updateSleep(0.5);
    armPosition = 3;
    turn(180);

    
    slideIsUp = true;
    move(250, 250);
    armPosition = 4;
    updateSleep(1.5);
    move(500, 300);
    
    runOutakeServo(0.6);
    
    //1st sample in bucket, going to pick up second
    
    armPosition = 3;
    move(-200, -200);
    slideIsUp = false;
    turn(185);
    

    
    armPosition = 2;
    move(0, 265);
    move(-100, 0);
    
    updateSleep(1);
    armPosition = 1;
    updateSleep(1);
    closeClaw();
    
    //2nd sample grabbed, going to bucket:
    
    updateSleep(0.5);
    armPosition = 4;
    
    updateSleep(0.5);
    turn(180);
    openClaw();

    armPosition = 3;
    slideIsUp = true;
    
    updateSleep(1.5);
    move(150, 480);
    runOutakeServo(0.7);
    /*    
    
    updateSleep(0.5);
    armPosition = 4;
    updateSleep(2);
    openClaw();
    updateSleep(0.5);
    
    turn(0);
    
    updateSleep(0.5);
    armPosition = 3;
    
    updateSleep(0.5);//move(0, 1, 300, 0.5);
    slideIsUp = true;
    move(1, 1, 400, 0.6);
    updateSleep(1.5);
    
    armPosition = 4;
    move(1, 1, 180, 0.3);
    updateSleep(2);
    runOutakeServo(1.475);
    
    updateSleep(1);
    
    
    //backs away from bucket - copy and pasted code ._.
    
    move(-1, -1, 500, 1);
    updateSleep(0.3);
    openClaw();
    armPosition = 3;
    slideIsUp = false;
    
    updateSleep(1.5);
    turn(180);
    
    move(0, 1, 100, 0.2);
    move(-1, 0, 300, 0.2);
    
    updateSleep(0.5);
    armPosition = 2;
    updateSleep(1);
    armPosition = 1;
    updateSleep(0.5);
    closeClaw();
    
    updateSleep(0.5);
    armPosition = 4;
    */
    
    // end of code we actually care about
  }
}