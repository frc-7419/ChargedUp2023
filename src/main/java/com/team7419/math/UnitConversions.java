package com.team7419.math;

public class UnitConversions {

    public static double roundToDecimal(double number, int n) {
        double power = Math.pow(10, n);
        return Math.round(number * power) / power;
    }

    public static double rpmToRadPerSec(double rpm) {
        return rpm * 2 * Math.PI / 60;
    }

    public static double mpsToRPM(double mps, double radius) { // m is meters
        return (60 * mps) / (2 * Math.PI * radius);
    }

    public static double rpmToMPS(double rpm, double radius) {
        return (2 * Math.PI * radius) / 60;
    }
    
    public static double mpsToRawSensorVelocity(double mps, double ticksPerRotation, double radius) {
        return rpmToRawSensorVelocity(mpsToRPM(mps, radius), ticksPerRotation);
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double metersToInches(double meters) { 
        return meters * 39.3701;
    }

    public static double rawSensorVelocityToRPM(double rawVelocity, double ticksPerRotation) {
        return rawVelocity * (1/ticksPerRotation) * 600;
    }

    public static double rawSensorVelocityToMPS(double rawSensorVelocity, double ticksPerRotation, double radius) {
        return rawSensorVelocity * (1/2048) * (2 * Math.PI * radius) * 0.1;
    }

    public static double rpmToRawSensorVelocity(double rpm, double ticksPerRotation) {
        return rpm * ticksPerRotation * (1/600);
    }

    public static double inchesToTicks(double inches, double radius, double gearRatioMultiplier,
            double ticksPerRotation) {
        return (ticksPerRotation * inches * gearRatioMultiplier) / (2 * Math.PI * radius);
    }

    public static double ticksToInches(double ticks, double radius, double gearRatioMultiplier,
            double ticksPerRotation) {
        return (2 * Math.PI * radius * ticks) / (ticksPerRotation * gearRatioMultiplier);
    }

    public static double thetaToInches(double theta, double radius) {
        return theta * (Math.PI / 180) * radius;
    }
}