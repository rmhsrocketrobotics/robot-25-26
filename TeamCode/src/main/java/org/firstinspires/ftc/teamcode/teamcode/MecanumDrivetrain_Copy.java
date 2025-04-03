// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


// public abstract class MecanumDrivetrain_Copy extends LinearOpMode {
//     private final double MOTOR_UNITS_CONSTANT = 100; // how many motor units are in an inch (TODO: find this value)
//     private final double INCHES_CONSTANT = 1 / MOTOR_UNITS_CONSTANT;

//     private MotorController xController; // controls left-right movement
//     private MotorController yController; // controls backward-forward movement
//     private MotorController rController; // controls rotation

//     private double targetXPosition;
//     private double targetYPosition;
//     private double targetRPosition;

//     private DcMotor frontLeft;
//     private DcMotor frontRight;
//     private DcMotor backLeft;
//     private DcMotor backRight;

//     private IMU imu;

//     private double lastXPosition = 0;
//     private double lastYPosition = 0;

//     private double absoluteXPosition = 0;
//     private double absoluteYPosition = 0;
    
//     public boolean usingMotorControllers = false;

//     public void initDrivetrain() {
//         xController = new MotorController(1, 0, 0);
//         yController = new MotorController(1, 0, 0);
//         rController = new MotorController(1, 0, 0);

//         // theres def a better way to do this but idc
//         frontLeft = hardwareMap.get(DcMotor.class, "fl");
//         frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//         frontRight = hardwareMap.get(DcMotor.class, "fr");
//         frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//         backLeft = hardwareMap.get(DcMotor.class, "bl");
//         backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//         backRight = hardwareMap.get(DcMotor.class, "br");
//         backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//         frontRight.setDirection(DcMotor.Direction.REVERSE); // do this for the motors so that they're all facing the right direction
//         backRight.setDirection(DcMotor.Direction.REVERSE);
        
//         // copy and pasted the following code (im pretty sure its magic)

//         // Retrieve the IMU from the hardware map
//         imu = hardwareMap.get(IMU.class, "imu");
//         // Adjust the orientation parameters to match your robot
//         IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                 RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                 RevHubOrientationOnRobot.UsbFacingDirection.UP));
//         // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//         imu.initialize(parameters);
//         imu.resetYaw();
//     }

//     public double getRobotHeading() {
//         return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//     }

//     public double toInches(double motorUnits) {
//         return motorUnits * INCHES_CONSTANT;
//     }

//     public double toMotorUnits(double inches) {
//         return inches * MOTOR_UNITS_CONSTANT;
//     }

//     public void setDrivetrainPowers(double x, double y, double r) { // r is short for rotation
//         // magic ahh code lmao
//         // you can do the calculations and stuff to derive these but they just move the robot so don't worry about it
//         frontLeft.setPower(y + x + r);
//         frontRight.setPower(y - x - r);
//         backLeft.setPower(y - x + r);
//         backRight.setPower(y + x - r);
//     }

//     public void setDrivetrainPowersAbsolute(double x, double y, double r) { // makes the robot move relative to the driver/field, not itself
//         double robotHeading = getRobotHeading();

//         double angleSine = Math.sin(- robotHeading);
//         double angleCosine = Math.cos(- robotHeading);

//         double rotatedX = x * angleCosine - y * angleSine;
//         double rotatedY = x * angleSine + y * angleCosine;

//         setDrivetrainPowers(rotatedX, rotatedY, r);
//     }

//     public void setDrivetrainTargetPositionAbsolute(double x, double y, double r) { // lets you set a position for the motors and let the pid do the work
//         targetXPosition = x;
//         targetYPosition = y;
//         targetRPosition = r; // r is in RADIANS
//     }

//     private void updatePosition() { // updates absolute position
//         // finds how far the robot has moved since we last checked (stored in xDelta and yDelta)
//         double currentXPosition = frontLeft.getCurrentPosition() + backRight.getCurrentPosition() - frontRight.getCurrentPosition() - backLeft.getCurrentPosition();
//         double currentYPosition = frontLeft.getCurrentPosition() + backRight.getCurrentPosition() + frontRight.getCurrentPosition() + backLeft.getCurrentPosition();

//         double xDelta = currentXPosition - lastXPosition;
//         double yDelta = currentYPosition - lastYPosition;

//         // gets the current robot heading and rotates the movement vectors so they add to the absolute position correctly
//         double robotHeading = getRobotHeading();

//         double angleSine = Math.sin(robotHeading);
//         double angleCosine = Math.cos(robotHeading);

//         double absoluteXDelta = xDelta * angleCosine - yDelta * angleSine;
//         double absoluteYDelta = xDelta * angleSine + yDelta * angleCosine;
        
//         absoluteXPosition += absoluteXDelta;
//         absoluteYPosition += absoluteYDelta;

//         // update the last position variables of the robot
//         lastXPosition = currentXPosition;
//         lastYPosition = currentYPosition;
//     }

//     private void updateDrivetrainPowers() {
//         double xPower = xController.updateMotor(absoluteXPosition, targetXPosition);
//         double yPower = yController.updateMotor(absoluteYPosition, targetYPosition);
//         double rPower = rController.updateMotor(getRobotHeading(), targetRPosition);
//         if (xPower == MotorController.DONE) {
//             xPower = 0;
//         }
//         if (yPower == MotorController.DONE) {
//             yPower = 0;
//         }
//         if (rPower == MotorController.DONE) {
//             rPower = 0;
//         }
        
//         setDrivetrainPowersAbsolute(xPower, yPower, rPower);
//     }

//     public void updateDrivetrain() { // call this in the event loop thing
//         updatePosition();
//         if (usingMotorControllers) {
//             updateDrivetrainPowers();
//         }
//     }
// }
