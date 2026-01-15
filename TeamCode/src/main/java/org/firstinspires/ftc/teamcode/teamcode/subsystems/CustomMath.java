package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import com.acmerobotics.roadrunner.Vector2d;

public class CustomMath {
    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
    }

    public static double distanceBetweenPoints(Vector2d point1, Vector2d point2) {
        double dx = point2.x - point1.x;
        double dy = point2.y - point1.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public static double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double dx = point2.x - point1.x;
        double dy = point2.y - point1.y;

        return Math.atan2(dy, dx);
    }

    public static double motionProfile(double maxAcceleration, double maxVelocity, double signedDistance, double elapsedTime) {
        // based on https://www.ctrlaltftc.com/advanced/motion-profiling

        double distance = Math.abs(signedDistance);
        double sign;

        if (signedDistance < 0) {
            sign = -1;
        } else {
            sign = 1;
        }

        // Calculate the time it takes to accelerate to max velocity
        double accelerationDeltaTime = maxVelocity / maxAcceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfwayDistance = distance / 2;
        double accelerationDistance = 0.5 * maxAcceleration * accelerationDeltaTime * accelerationDeltaTime;

        if (accelerationDistance > halfwayDistance) {
            accelerationDeltaTime = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * accelerationDeltaTime * accelerationDeltaTime;

        // recalculate max velocity based on the time we have to accelerate and decelerate
        maxVelocity = maxAcceleration * accelerationDeltaTime;

        // we decelerate at the same rate as we accelerate
        double decelerationDeltaTime = accelerationDeltaTime;

        // calculate the time that we're at max velocity
        double cruiseDistance = distance - (2 * accelerationDistance);
        double cruiseDeltaTime = cruiseDistance / maxVelocity;
        double decelerationTime = accelerationDeltaTime + cruiseDeltaTime;

        /// all of the above stays the same for a single path, all of the below changes based on elapsedTime

        // check if we're still in the motion profile
        double entireDeltaTime = accelerationDeltaTime + cruiseDeltaTime + decelerationDeltaTime;
        if (elapsedTime > entireDeltaTime) {
            return distance * sign;
        }

        // if we're accelerating
        if (elapsedTime < accelerationDeltaTime) {
            // use the kinematic equation for acceleration
            return (0.5 * maxAcceleration * elapsedTime * elapsedTime) * sign;
        }

        // if we're cruising
        if (elapsedTime < decelerationTime) {
            accelerationDistance = 0.5 * maxAcceleration * accelerationDeltaTime * accelerationDeltaTime;
            double cruiseCurrentDeltaTime = elapsedTime - accelerationDeltaTime;

            // use the kinematic equation for constant velocity
            return (accelerationDistance + (maxVelocity * cruiseCurrentDeltaTime)) * sign;
        }

        // if we're decelerating
        accelerationDistance = 0.5 * maxAcceleration * accelerationDeltaTime * accelerationDeltaTime;
        cruiseDistance = maxVelocity * cruiseDeltaTime;
        decelerationTime = elapsedTime - decelerationTime;

        // use the kinematic equations to calculate the instantaneous desired position
        return (accelerationDistance + cruiseDistance + (maxVelocity * decelerationTime) - (0.5 * maxAcceleration * decelerationTime * decelerationTime)) * sign;
    }

    public static double odometryPodTicksToInches(double ticks) {
        return ticks / 504.9;
    }

    public static Vector2d rotatePointAroundOrigin(Vector2d point, double angle) {
        double rotatedX = (point.x * Math.cos(angle)) - (point.y * Math.sin(angle));
        double rotatedY = (point.x * Math.sin(angle)) + (point.y * Math.cos(angle));
        return new Vector2d(rotatedX, rotatedY);
    }
}