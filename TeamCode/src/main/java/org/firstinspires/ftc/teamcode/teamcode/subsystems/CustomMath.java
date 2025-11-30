package org.firstinspires.ftc.teamcode.teamcode.subsystems;

public class CustomMath {
    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
    }
}