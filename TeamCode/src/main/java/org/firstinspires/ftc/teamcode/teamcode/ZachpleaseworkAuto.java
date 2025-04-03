// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


// public class ZachpleaseworkAuto {


// private void start(){
// opModeTypeCrap();
// startMoving(0,2,0,1000);
// }
// public void opModeTypeCrap(){
//     DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left_Motor");
//     DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right_Motor");
//     DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left_Motor");
//     DcMotor = hardwareMap.get(DcMotor.class,"back_right_Motor");
    
//     resetMotor(frontLeft);
//     resetMotor(frontRight);
//     resetMotor(backLeft);
//     resetMotor(backRight);
//   }
    
// private void resetMotor(DcMotor motor){
//     motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
//   }

// private void startMoving(double x, double y, double r, int ticks) {
//   int ticksTime = ticks * 10;//chage the 10 based on testing. Its time btw
//     int speed = 1;
//     double magnitude = Math.sqrt(x * x + y * y);
//     if (magnitude > 0) {  // Prevent division by zero
//         x /= magnitude;
//         y /= magnitude;
//     }

//     double frontLeftPower = (y + x + r)*speed;
//     double frontRightPower = (y - x - r)*speed;
//     double backLeftPower = (y - x + r)*speed;
//     double backRightPower = (y + x - r)*speed;

//     double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
//             Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
//     if (max > 1.0) { //this makes it so that if the value exceeds 1 (bad) it gets divided by itself and goes back to one like a good boy
//         frontLeftPower /= max;
//         frontRightPower /= max;
//         backLeftPower /= max;
//         backRightPower /= max;
//     }    

//     frontLeft.setPower((frontLeftPower));  
//     frontRight.setPower((frontRightPower));
//     backLeft.setPower((backLeftPower));
//     backRight.setPower((backRightPower));

//     while (opModeIsActive() &&
//             (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
//     }

//     stopAllMotors(); //IF IT CANT SET POWER WHAT DO I DO YO
// }
// }

