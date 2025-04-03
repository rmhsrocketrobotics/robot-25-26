package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorController {
    /* 
     * requires > import com.qualcomm.robotcore.util.ElapsedTime;
     * 
     * should be implemented kinda like this
     * > motorPower = motorContoller.updateMotor(motorPosition, [desired position]);
     * > if (motorPower == MotorController.DONE) {break;}
     * 
     * is this code bad? probably yeah, but i wrote it so freak you
     * 
     */

    //public static final byte DONE = 69;

    public static final byte BangBang = 0;
    public static final byte Proportional = 1;
    public static final byte PID = 2;

    //double tolerance = 10; //how close the motor has to get to its target to stop
    //double secondsToEnd = 0.07; //how long the motor has to be within the tolerance to stop

    //ElapsedTime secondsWithinTolerance;
    
    byte mode;
    
    /* 
     * the following variables are only used for proportional and pid
     */
    
    double Kp; //proportional term

    /* 
     * the following variables are only used for pid
     */

    double Ki; //integral term
    double integralSum; //used for calculating the integral

    double Kd; //derivative term
    double lastError; //used for calculating the derivative
    
    ElapsedTime timer;
    

    private static double bound(double num, double lowerBound, double upperBound) {
        return Math.max(Math.min(num, upperBound), lowerBound);
    }

    MotorController() { //assumes you want a bang bang controller
        //this.secondsWithinTolerance = new ElapsedTime();
    
        this.mode = MotorController.BangBang;
    }

    MotorController(double Kp) { //assumes you want a proportional controller
        //this.secondsWithinTolerance = new ElapsedTime();
    
        this.mode = MotorController.Proportional;
        this.Kp = Kp;
    }

    MotorController(double Kp, double Ki, double Kd) { //assumes you want a PID controller
        //this.secondsWithinTolerance = new ElapsedTime();
    
        this.mode = MotorController.PID;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.integralSum = 0;
        this.lastError = 0;

        this.timer = new ElapsedTime();
    }

    MotorController(double Kp, double Ki, double Kd, double tolerance, double secondsToEnd) { //assumes you want a PID controller but also sets the tolerance and secondsToEnd
        //this.secondsWithinTolerance = new ElapsedTime();
    
        this.mode = MotorController.PID;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.integralSum = 0;
        this.lastError = 0;

        this.timer = new ElapsedTime();

        //this.tolerance = tolerance;
        //this.secondsToEnd = secondsToEnd;
    }

    private double BangBangController(double currentPosition, double reference) {
        double error = reference - currentPosition;
        
        if (error > 1) {
            return 1;
        } else if (error < -1) {
            return -1;
        } else {
            return 0;
        }
    }

    private double ProportionalController(double currentPosition, double reference) {
        double error = reference - currentPosition;

        return bound(error * Kp, -1, 1);
    }

    private double PIDController(double currentPosition, double reference) {
        double error = reference - currentPosition;

        //the sum of change over time
        integralSum += error * timer.seconds();

        //the rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

        lastError = error;

        timer.reset();

        return bound((error * Kp) + (integralSum * Ki) + (derivative * Kd), -1, 1);
    }
    

    public double updateMotor(double currentPosition, double reference) {
        double error = reference - currentPosition;
        double output = 0;
        
        switch (mode) {
            case BangBang:
                output = BangBangController(currentPosition, reference);
                break;
        
            case Proportional:
                output = ProportionalController(currentPosition, reference);
                break;
            
            case PID:
                output = PIDController(currentPosition, reference);
                break;
            
            default: //this should never run, if this runs things are probably very bad
                System.out.println("bad things have happened in the MotorController class ._.");
                break;
            
        }
        
        // if (! (error < tolerance && error > -tolerance)) {
        //     secondsWithinTolerance.reset();
        // }
        // if (secondsWithinTolerance.seconds() >= secondsToEnd) {
        //     return 0;
        // }
        
        return output;
    }
}