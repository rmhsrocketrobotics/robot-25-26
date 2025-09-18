package org.firstinspires.ftc.teamcode.LastYearExampleCode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    /* 
     * requires import com.qualcomm.robotcore.util.ElapsedTime;
     * 
     * to implement, create the object with: PIDController armController = new PIDController;
     * then use in a loop: armMotor.setPower(armController.update(currentPosition, desiredPosition))
     * and break when completed is true: if (armController.completed) {break;}
     */

    double tolerance = 10; //how close the motor has to get to its target to stop
    double secondsToEnd = 0.07; //how long the motor has to be within the tolerance to stop

    ElapsedTime secondsWithinTolerance; //tracks how long the motor has been within the tolerance; if it's been longer than secondsToEnd, then make this.done true
    
    double Kp; //proportional term
    double Ki; //integral term
    double Kd; //derivative term

    double integralSum; //used for calculating the integral
    double lastError; //used for calculating the derivative
    
    ElapsedTime secondsSinceLastUpdated; //used for calculating the integral and derivative

    public boolean completed = false; //becomes true when the pid controller is within it's tolerance for secondsToEnd seconds
    

    public static double bound(double num, double lowerBound, double upperBound) {
        return Math.max(Math.min(num, upperBound), lowerBound);
    }


    PIDController(double Kp, double Ki, double Kd, double tolerance, double secondsToEnd) { //constructor moment
        this.tolerance = tolerance;
        this.secondsToEnd = secondsToEnd;

        this.secondsWithinTolerance = new ElapsedTime();
    
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.integralSum = 0;
        this.lastError = 0;

        this.secondsSinceLastUpdated = new ElapsedTime();

        this.completed = false;
    }


    private double getPIDControllerOutput(double currentPosition, double reference) { //where the actual pid stuff happens
        double error = reference - currentPosition;

        //the sum of change over time
        integralSum += error * secondsSinceLastUpdated.seconds();

        //the rate of change of the error
        double derivative = (error - lastError) / secondsSinceLastUpdated.seconds();

        lastError = error;

        secondsSinceLastUpdated.reset();

        return bound((error * Kp) + (integralSum * Ki) + (derivative * Kd), -1, 1);
    }
    

    public double update(double currentPosition, double reference) { //call this in a loop
        double error = reference - currentPosition;

        boolean errorIsWithinTolerance = Math.abs(error) < tolerance;

        if (!errorIsWithinTolerance) {
            secondsWithinTolerance.reset();
        }

        completed = secondsWithinTolerance.seconds() >= secondsToEnd; //sets completed to true when it is within tolerance for the desired length of time
        
        double output = getPIDControllerOutput(currentPosition, reference);
        
        return output;
    }

    public void reset() { //resets the PID controller - do this when you're changing references
        secondsWithinTolerance.reset();
        integralSum = 0;
        lastError = 0;
        secondsSinceLastUpdated.reset();
        completed = false;
    }
}
