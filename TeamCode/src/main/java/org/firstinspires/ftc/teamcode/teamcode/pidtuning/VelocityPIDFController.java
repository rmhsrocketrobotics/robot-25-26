package org.firstinspires.ftc.teamcode.teamcode.pidtuning;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.util.MovingStatistics;

public class VelocityPIDFController {
    private static final int ACCEL_SAMPLES = 3;
    private static final double VELOCITY_EPSILON = 20 + 1e-6;

    private PController controller;
    private SimpleMotorFeedforward feedforward;

    private MovingStatistics accelSamples;

    private double lastPosition = Double.NaN;
    private double lastVelocity = Double.NaN;

    private double kD;
    private double kV;
    private double kA;
    private double kStatic;

    public VelocityPIDFController(double kP, double kD, double kV, double kA, double kStatic) {
        controller = new PController(kP);
        feedforward = new SimpleMotorFeedforward(kStatic, kV, kA);

        accelSamples = new MovingStatistics(ACCEL_SAMPLES);

        this.kD = kD;

        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;

        reset();
    }

    private double calculateAccel(double measuredPosition, double measuredVelocity) {
        double dx = measuredPosition - lastPosition;
        if (dx != 0.0 && Math.abs(measuredVelocity - lastVelocity) > VELOCITY_EPSILON) {
            double accel = (measuredVelocity * measuredVelocity - lastVelocity * lastVelocity) / (2.0 * dx);

            lastPosition = measuredPosition;
            lastVelocity = measuredVelocity;

            accelSamples.add(accel);
        } else {
            accelSamples.add(0.0);
        }

        return accelSamples.getMean();
    }

    public double update(double measuredPosition, double measuredVelocity,
                         double targetVelocity, double targetAcceleration) {
        if (Double.isNaN(lastPosition)) {
            lastPosition = measuredPosition;
        }
        if (Double.isNaN(lastVelocity)) {
            lastVelocity = measuredVelocity;
        }

        double accel = calculateAccel(measuredPosition, measuredVelocity);

        double correctionValue = controller.calculate(measuredVelocity, targetVelocity);
        double derivativeValue = kD * (targetAcceleration - accel);
        double feedforwardValue = feedforward.calculate(targetVelocity, targetAcceleration);

        return correctionValue + derivativeValue + feedforwardValue;
    }

    public void reset() {
        controller.reset();

        lastPosition = Double.NaN;
        lastVelocity = Double.NaN;
    }
}