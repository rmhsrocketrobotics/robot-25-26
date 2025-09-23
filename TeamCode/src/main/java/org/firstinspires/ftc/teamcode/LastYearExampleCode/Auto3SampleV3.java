package org.firstinspires.ftc.teamcode.LastYearExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled

public class Auto3SampleV3 extends MecanumDrivetrain3{
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
      armMotorTargetPosition = degreesToMotorUnits(-2);
      
    } else if (armPosition == 2) {
      armMotorTargetPosition = degreesToMotorUnits(8);
      
    } else if (armPosition == 3) {
      armMotorTargetPosition = degreesToMotorUnits(70);
      
    } else if (armPosition == 4) {
      armMotorTargetPosition = degreesToMotorUnits(109); //og was 105
      
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
    
    //treat this like a graph, positive x is right, positive y is up (forward)
    @Override
    public void move(double xTicks, double yTicks, double degrees, double speed) { // lets you set a position for the motors and let the pid do the work
        xTicks = -xTicks;
        yTicks = -yTicks;
        
        double rTicks = degrees * 10.76; //insert conversion factor here
        //double rTicks = degrees * 11; //insert conversion factor here

        double flTarget = frontLeft.getCurrentPosition() + yTicks + xTicks + rTicks;
        double frTarget = frontRight.getCurrentPosition() + yTicks - xTicks - rTicks;
        double blTarget = backLeft.getCurrentPosition() + yTicks - xTicks + rTicks;
        double brTarget = backRight.getCurrentPosition() + yTicks + xTicks - rTicks;
        
        // if (degrees == 0 && xTicks == 0) {
        //   flTarget += 17;
        //   blTarget += 17;
        // }
        //brTarget = brTarget + (Math.abs(brTarget) * 0.1)

        while (opModeIsActive() && (!motorsAreDone())) {
            double flPower = flPID.update(frontLeft.getCurrentPosition(), flTarget);
            double frPower = frPID.update(frontRight.getCurrentPosition(), frTarget);
            double blPower = blPID.update(backLeft.getCurrentPosition(), blTarget);
            double brPower = brPID.update(backRight.getCurrentPosition(), brTarget);
            
            frontLeft.setPower(PIDController.bound(flPower, -speed, speed));
            frontRight.setPower(PIDController.bound(frPower, -speed, speed));
            backLeft.setPower(PIDController.bound(blPower, -speed, speed));
            backRight.setPower(PIDController.bound(brPower, -speed, speed));
            
            telemetry.addData("frontLeft.getCurrentPosition()", frontLeft.getCurrentPosition());
            telemetry.addData("flTarget)", flTarget);
            updateNonDrivetrain();
            
        }
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
      flPID.reset();
      frPID.reset();
      blPID.reset();
      brPID.reset();
    }
    
    @Override
    public void move(double xTicks, double yTicks) {
        move(xTicks, yTicks, 0, 0.5); //og speed value: 0.5
    }
    
    @Override
    public void turn(double degrees) {
        move(0, 0, degrees, 0.69); //og speed value: 0.75
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
    
    // while (opModeIsActive()) {
    //   armPosition = 4;
    //   updateSleep(1);
    //   openClaw();
    //   updateSleep(1);
    //   armPosition = 3;
    //   updateSleep(2);
    //   armPosition = 2;
    //   updateSleep(2);
    //   armPosition = 1;
    //   updateSleep(2);
    //   closeClaw();
    //   updateSleep(1);
    // }
    
    
    //>>>>CODE WE ACTUALLY CARE ABOUT:<<<<
    // start to basket:
    armPosition = 3;
    move(0, -100);
    slideIsUp = true;
    
    move(850, 0);
    updateSleep(1);
    armPosition = 4;
    move(175, 75);
    //move(0, 40);
    
    runOutakeServo(0.6);
    
    // basket to 1st sample:
    
    armPosition = 3;
    move(-450, -440);
    slideIsUp = false;
    
    armPosition = 2;
    turn(180);
    move(0, 120);
    //updateSleep(0.5);
    
    
    //move(0, -100);
    //updateSleep(0.5);
    armPosition = 1;
    updateSleep(0.75);
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
    move(250, 370);
    armPosition = 4;
    updateSleep(1.5);
    move(100, 200);
    
    runOutakeServo(0.6);
    
    //1st sample in bucket, going to pick up second
    
    armPosition = 3;
    move(-80, -110);
    slideIsUp = false;
    turn(190);
    

    
    armPosition = 2;
    move(0, 355); //!!
    //move(-100, 0);
    
    updateSleep(0.5);
    armPosition = 1;
    updateSleep(0.75);
    closeClaw();
    
    //2nd sample grabbed, going to bucket:
    
    updateSleep(0.5);
    armPosition = 4;
    
    move(310, 0);
    turn(180);
    openClaw();

    armPosition = 3;
    updateSleep(0.5);
    slideIsUp = true;
    
    updateSleep(1.5);
    move(370, 530);
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