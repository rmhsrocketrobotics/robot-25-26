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

    static double angleBetweenPoints(Vector2d point1, Vector2d point2) {
        double dx = point2.x - point1.x;
        double dy = point2.y - point1.y;

        return Math.atan2(dy, dx);
    }
}