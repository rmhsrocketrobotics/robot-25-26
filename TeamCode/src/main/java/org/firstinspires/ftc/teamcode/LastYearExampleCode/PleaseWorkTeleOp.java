package org.firstinspires.ftc.teamcode.LastYearExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
// (90, -333) and (0, -1566)
@Disabled//@TeleOp

public class PleaseWorkTeleOp extends MecanumDrivetrain{
  private Gamepad gamepad2Last = new Gamepad();
  
  //put the definitions for the motors and stuff here:
  //eg. private Motor leftMotor;
  private Servo rightClaw;
  private Servo leftClaw;
  
  private CRServo outakeServo;
  
  private DcMotor armMotor;
  private ElapsedTime armMotorBeingReset = new ElapsedTime(10);
  private MotorController armMotorController;
  private double armMotorTargetPosition;
  
  private ElapsedTime transferingSampleTimer = new ElapsedTime(10);
  
  private byte armPosition = 4;
  
  boolean rightBumperPressedLast = false;
  
  private MotorController slideController = new MotorController(0.02, 0, 0.0003);
  
  boolean downPressedLast = false;
  boolean upPressedLast = false;
  
  boolean aPressedLast = false;
  boolean bPressedLast = false;
  boolean yPressedLast = false;
  boolean hangPushingDown = false;
  
  boolean leftBumperPressedLast = false;
  
  double slideTargetPosition;
  
  boolean PIDControlled = false;
  boolean manualOverride = false;
  boolean manualOverride2 = false;
  
  double targetRadians = 0;
  
  boolean clawIsOpen = false;
  
  boolean slideIsUp = false;
  
  double slideMotorFF;
  
  private DcMotor slideMotor;
  private MotorController slideMotorController;
  private double slideMotorTargetPosition;
  
  private DcMotor hangMotor1;
  private DcMotor hangMotor2;
  
  ElapsedTime timer;
  
  ElapsedTime outakeServoTimer = new ElapsedTime(6969);
  double outakeServoDirection;
  
  private boolean usingAbsoluteMovement = false;
  
  ElapsedTime accelerationTimer;
  
  private RotationController rotationController = new RotationController(3, 0, 0.1, 0.03, 0.2);
  
  // private void resetArmEncoder() {
  //   armMotorBeingReset.reset();
  //   armMotor.setPower(-0.4);
  // }
  
  private double motorUnitsToDegrees(double motorUnits) {
    return (0.073 * motorUnits) + 114.309;
  }
  
  private double degreesToMotorUnits(double degrees) {
    return (13.7 * degrees) - 1566;
  }
  
  // controlled using start + B
  private void controller2() {
    if (gamepad2.right_bumper && (!rightBumperPressedLast)) {
      clawIsOpen = !clawIsOpen;
    }
    
    rightBumperPressedLast = gamepad2.right_bumper;
    
    
    
    if (clawIsOpen) {
      rightClaw.setPosition(0.4);
      leftClaw.setPosition(0.243);
    } else {
      rightClaw.setPosition(0.233);
      leftClaw.setPosition(0.426);
    }
    
    // leftClaw.setPosition(leftClaw.getPosition() + (gamepad2.left_stick_y * 0.01));
    // rightClaw.setPosition(rightClaw.getPosition() + (gamepad2.left_stick_x * 0.01));
    // telemetry.addData("left claw position", leftClaw.getPosition());
    // telemetry.addData("right claw position", rightClaw.getPosition());
    
    // if ((!gamepad2.a) && gamepad2.left_trigger > 0.1 && outakeServoTimer.seconds() > 0.8){
    //   outakeServoTimer.reset();
    //   outakeServoDirection = -1;
      
    // } else if ((!gamepad2.a) && gamepad2.right_trigger > 0.1 && outakeServoTimer.seconds() > 0.8) {
    //   outakeServoTimer.reset();
    //   outakeServoDirection = 1;
    // }
      
    // if (outakeServoTimer.seconds() < 0.23) {
    //   outakeServo.setPower(outakeServoDirection);
    // } else {
    //   outakeServo.setPower(0);
    // }
    // if (outakeServoTimer.seconds() > 0.23) {
    //   if (gamepad2.a && gamepad2.left_trigger > 0.1) {
    //     outakeServo.setPower(-gamepad2.left_trigger);
    //   } else if (gamepad2.a && gamepad2.right_trigger > 0.1) {
    //     outakeServo.setPower(gamepad2.right_trigger);
    //   } else {
    //     outakeServo.setPower(0);
    //   }
    // }
    
    if (gamepad2.right_trigger != 0) {
      if (gamepad2.right_trigger < 0.8) {
        outakeServo.setPower(0.15);
      } else {
        outakeServo.setPower(gamepad2.right_trigger);
      }
      
    } else if (gamepad2.left_trigger != 0) {
      if (gamepad2.left_trigger < 0.8) {
        outakeServo.setPower(-0.15);
      } else {
        outakeServo.setPower(- gamepad2.left_trigger);
      } 
      
    } else if (gamepad2.left_bumper) {
      outakeServo.setPower(-0.15);
      
    } else {
      outakeServo.setPower(0);
    }
    
    //arm code (why is this so complicated ahhhhh)
    double armMotorFFPower = Math.cos(Math.toRadians(motorUnitsToDegrees(armMotor.getCurrentPosition())));
    armMotorFFPower *= 0.3; //change this constant to make it good
    // if (armMotor.getCurrentPosition() < 10) {
    //   armMotorFFPower = 0;
    // }
    
    double armMotorPIDPower = armMotorController.updateMotor(armMotor.getCurrentPosition(), armMotorTargetPosition);
    
    if (Math.abs(armMotorPIDPower) < 0.05) {
      armMotorPIDPower = 0;
      telemetry.addLine("arm done");
    } 
    
    // if (armMotorBeingReset.seconds() > 4) {
    //   armMotor.setPower(armMotorPIDPower * 0.4);
    // } else if (armMotorBeingReset.seconds() > 3) {
    //   armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //   armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //   armMotor.setPower(0);
    // }
    armMotor.setPower(gamepad2.left_stick_y * -0.4);
    
    telemetry.addData("amrMotorBeingREset.seconds()", armMotorBeingReset.seconds());
    // telemetry.addData("arm m otor power", );
    // telemetry.addData("armMotorTargetPosition", armMotorTargetPosition);
    // telemetry.addData("armMotorFFPower", armMotorFFPower);
    // telemetry.addData("armMotorPIDPower", armMotorPIDPower);
    
    
    
    // if (gamepad2.dpad_up && !upPressedLast) {
    //   armPosition += 1;
    // }
    // else if (gamepad2.dpad_down && !downPressedLast) {
    //   armPosition -= 1;
    // }
    
    // upPressedLast = gamepad2.dpad_up;
    // downPressedLast = gamepad2.dpad_down;
    
    // if (armPosition < 1) {
    //   armPosition = 1;
    // }
    
    // if (armPosition > 4) {
    //   armPosition = 4;
    // }
    
    // if (armPosition == 1) {
    //   armMotorTargetPosition = degreesToMotorUnits(7);
      
    // } else if (armPosition == 2) {
    //   armMotorTargetPosition = degreesToMotorUnits(20);
      
    // } else if (armPosition == 3) {
    //   armMotorTargetPosition = degreesToMotorUnits(85);
      
    // } else if (armPosition == 4) {
    //   armMotorTargetPosition = degreesToMotorUnits(105);
      
    // }
    // if (gamepad2.y) {
    //   armMotor.setPower(gamepad2.right_stick_x);
    // }
    
    telemetry.addData("arm position", armPosition);
    
    // if (gamepad2.a) {
    //   slideMotorTargetPosition = 100;
      
    // } else if (gamepad2.b) {
    //   slideMotorTargetPosition = 0;
    // }
    
  // if (gamepad2.x) {
  //   manualOverride2 = true;
  // }
    
  // if (gamepad2.left_bumper && (!leftBumperPressedLast)) {
  //   slideIsUp = !slideIsUp;
  // }


  // if (!manualOverride2) {
  
  //     if (slideIsUp) {
  //         slideTargetPosition = -4400;
  //     } else {
  //         slideTargetPosition = 0;
  //     }
         
  //     double slideMotorPIDPower = slideController.updateMotor(slideMotor.getCurrentPosition(), slideTargetPosition);
  
  
  //     if (Math.abs(slideMotorPIDPower) < 0.1) {
  //         slideMotorPIDPower = 0;
  //     }
  
  
  //     if (slideMotorPIDPower > 0 && slideMotor.getCurrentPosition() > -10) {
  //         slideMotorPIDPower = 0;
  //     }
  
  
  //     slideMotor.setPower(slideMotorPIDPower);
  
  
  // } else {
  //     slideMotor.setPower(gamepad2.right_stick_y * 1);
  // }
  if (slideMotor.getCurrentPosition() < -4000) {
    slideMotorFF = -0.1;
  } else {
    slideMotorFF = 0;
  }
  slideMotor.setPower(gamepad2.right_stick_y + slideMotorFF);
  
  leftBumperPressedLast = gamepad2.left_bumper;
    
    // double slideMotorPIDPower = slideMotorController.updateMotor(slideMotor.getCurrentPosition(), slideMotorTargetPosition);
    // slideMotor.setPower(slideMotorPIDPower);
    if (transferingSampleTimer.seconds() < 5) {
      if (transferingSampleTimer.seconds() > 0) { // this code is bad but idc
        armPosition = 4;
        clawIsOpen = false;
      }
      if (transferingSampleTimer.seconds() > 2) {
        clawIsOpen = true;
      }
      if (transferingSampleTimer.seconds() > 2.5) {
        armPosition = 3;
      }
    }
    
    // if (gamepad2.y && (!yPressedLast)) {
    //   armMotorBeingReset.reset();
    // }
    
    yPressedLast = gamepad2.y;
    
    if (gamepad2.y && (!bPressedLast)) {
      hangPushingDown = !hangPushingDown;
    }
    
    bPressedLast = gamepad2.y; // lmfao
    
    // if (gamepad2.a && (!aPressedLast)) {
      
    // }
    
    // aPressedLast = gamepad2.a;
    
    if (hangPushingDown) {
      hangMotor1.setPower(0.6);
      hangMotor2.setPower(0.6);
    } else {
      if (gamepad2.dpad_up) {
        hangMotor1.setPower(-1);
        hangMotor2.setPower(-1);
      } else if (gamepad2.dpad_down) {
        hangMotor1.setPower(1);
        hangMotor2.setPower(1);
      } else {
        hangMotor1.setPower(0);
        hangMotor2.setPower(0);
      }
    }
  }
  
  private void controller1() {
    
    if (gamepad1.y) {
      imu.resetYaw();
    }

    if (gamepad1.x) {
        manualOverride = true;
    }

    if (gamepad1.dpad_up) {
        PIDControlled = true;
        targetRadians = 0;


    } else if (gamepad1.dpad_right) {
        PIDControlled = true;
        targetRadians = Math.PI * 1.5;


    } else if (gamepad1.dpad_down) {
        PIDControlled = true;
        targetRadians = Math.PI * 1;


    } else if (gamepad1.dpad_left) {
        PIDControlled = true;
        targetRadians = Math.PI * 0.5;
    }


    if (PIDControlled && (!manualOverride)) {
        double[] powers = rotationController.updateMotor(getRobotHeading(), targetRadians);
       
        if (Arrays.equals(powers, rotationController.DONE)) {
            PIDControlled = false;


        } else {
            double rotationPower = powers[0] + powers[1] + powers[2];
       
            setDrivetrainPowers(0, 0, -rotationPower);
        }
   
    } else {
        float right_stick_x = funnyPiecewiseFunction(gamepad1.right_stick_x);
        float right_stick_y = funnyPiecewiseFunction(gamepad1.right_stick_y);
        float left_stick_x = funnyPiecewiseFunction(gamepad1.left_stick_x);

      //allows for the slow movement thing
        if (gamepad1.left_bumper){ //fast mode
        right_stick_x *= 1;
        right_stick_y *= 1;
        left_stick_x *= 1;
       
        } else if (gamepad1.right_bumper) { //slow mode
        right_stick_x *= 0.3F;
        right_stick_y *= 0.3F;
        left_stick_x *= 0.3F;
       
        } else { //normal mode
        right_stick_x *= 0.75F;
        right_stick_y *= 0.6F;
        left_stick_x *= 0.8F;
        }
        setDrivetrainPowers(right_stick_x, -right_stick_y, left_stick_x);
    }
}

  
  //executed when this OpMode is selected from the Driver Station
  @Override
  public void runOpMode() { //will run once you press init
    rightClaw = hardwareMap.get(Servo.class, "lc");
    leftClaw = hardwareMap.get(Servo.class, "rc");
    
    outakeServo = hardwareMap.get(CRServo.class, "outakeServo");
    
    armMotor = hardwareMap.get(DcMotor.class, "arm");
    armMotorController = new MotorController(0.04, 0, 0.0006);
    armMotorTargetPosition = 0;
    
    hangMotor1 = hardwareMap.get(DcMotor.class, "hangMotor1");
    hangMotor2 = hardwareMap.get(DcMotor.class, "hangMotor2");
    
    slideMotor = hardwareMap.get(DcMotor.class, "slide");
    slideMotorController = new MotorController(0.0005, 0, 0);
    slideMotorTargetPosition = 0;
    
    
    //MotorController armMotorController = MotorController(0.1);
    
    //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    hangMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    hangMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // armMotor.setPower(0.1);

    timer = new ElapsedTime(10);
    //accelerationTimer = new ElapsedTime();
    
    
    initDrivetrain();
    
    //setAccelerationValues(15, 20, 10, 5, 5, 5);
    
    waitForStart(); //the following code will run once you press the triangle play button thing
    
    //resetArmEncoder();
    
    while (opModeIsActive()) { //runs in a loop while op mode is running
      controller1();
      controller2(); 
      
      // telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);
      // telemetry.addData("gamepad2.right_stick_x", gamepad2.right_stick_x);
      // telemetry.addData("gamepad2.left_stick_y", gamepad2.left_stick_y);
      telemetry.addData("armMotor.getCurrentPosition()", armMotor.getCurrentPosition());
      telemetry.addData("armMotor.getCurrentPosition() converted to degrees", motorUnitsToDegrees(armMotor.getCurrentPosition()));
      // telemetry.addData("armMotorTargetPosition", armMotorTargetPosition);
      telemetry.addData("robot rotation", getRobotHeading());
      
      telemetry.addData("slideMotor.getCurrentPosition()", slideMotor.getCurrentPosition());

      telemetry.update();
      
      gamepad2Last.copy(gamepad2);
      
      timer.reset();
    }
  }
  // alr bro like ts is hard to explain js look at this desmos graph imo https://www.desmos.com/calculator/2tmtqifqo3
  public static float funnyPiecewiseFunction(float power){
    float deadzone = 0.1F;
    float startPower = 0.3F;

    if (power > 1) {power = 1;}
    else if (power < -1) {power = -1;}

    if (power < -deadzone){
      return lineBetweenPoints(-deadzone, -startPower, -1, -1, power);

    } else if (power > deadzone) {
      return lineBetweenPoints(deadzone, startPower, 1, 1, power);

    } else {
      return 0;
    }
  }

  public static float lineBetweenPoints(float x1, float y1, float x2, float y2, float x){
    float slope = (y1-y2)/(x1-x2);
    float yIntercept = y1 - (slope * x1);

    return (slope * x) + yIntercept;
  }
}


