package com.team7419.math;

public class UnitConversions {
  /**
   * Rounds a double value to a specified amount of decimal places.
   * @param number Double to round
   * @param n Number of decimal places to round to (positive = more precise)
   * @return The rounded result
   */
  public static double roundToDecimal(double number, int n) {
    double power = Math.pow(10, n);
    return Math.round(number * power) / power;
  }
  /**
   * Converts rotations per minute to radians per second
   * @param rpm Rotational speed, in rotations per minute, to convert
   * @return Converted rotational speed, expressed in radians per second
   */
  public static double rpmToRadPerSec(double rpm) {
    return rpm * 2 * Math.PI / 60;
  }
  /**
   * Converts linear speed in meters per second to rotational speed in rotations per minute
   * @param mps Linear speed, in meters per second, to convert
   * @param radius Radius of the rotating mechanism
   * @return Converted rotational speed, expressed in rotations per minute
   */
  public static double mpsToRPM(double mps, double radius) { // m is meters
    return (60 * mps) / (2 * Math.PI * radius);
  }
  /**
   * Converts rotational speed in rotations per minute to linear speed in meters per second
   * @param rpm Rotational speed, in rotations per minute, to convert
   * @param radius Radius of the rotating mechanism
   * @return Converted linear speed, expressed in meters per second
   */
  public static double rpmToMPS(double rpm, double radius) {
    return (2 * Math.PI * radius) / 60;
  }
  /**
   * Converts linear speed in meters per second to raw sensor velocity (ticks per second)
   * @param mps Linear speed, in meters per second, to convert
   * @param ticksPerRotation Number of ticks counted by the sensor when the mechanism rotates once
   * @param radius Radius of the rotating mechanism
   * @return Converted rotational speed, expressed in raw sensor velocity
   */
  public static double mpsToRawSensorVelocity(double mps, double ticksPerRotation, double radius) {
    return rpmToRawSensorVelocity(mpsToRPM(mps, radius), ticksPerRotation);
  }
  /**
   * Converts a length in inches to meters
   * @param inches Length, in inches, to convert
   * @return Converted length, expressed in meters
   */
  public static double inchesToMeters(double inches) {
    return inches * 0.0254;
  }
  /**
   * Converts a length in meters to inches
   * @param meters Length, in meters, to convert
   * @return Converted length, expressed in inches
   */
  public static double metersToInches(double meters) {
    return meters * 39.3701;
  }
  /**
   * Converts rotational speed in raw sensor units to rotations per minute
   * @param rawVelocity Rotational speed, in raw sensor units, to convert
   * @param ticksPerRotation Number of ticks counted by the sensor when the mechanism rotates once
   * @return Converted rotational speed, expressed in raw sensor velocity
   */
  public static double rawSensorVelocityToRPM(double rawVelocity, double ticksPerRotation) {
    return rawVelocity * (1 / ticksPerRotation) * 600;
  }
  /**
   * Converts rotational speed in raw sensor units to linear speed in meters per second
   * @param rawSensorVelocity Rotational speed, in raw sensor units, to convert
   * @param ticksPerRotation Number of ticks counted by the sensor when the mechanism rotates once
   * @param radius Radius of the rotating mechanism
   * @return Converted linear speed, expressed in meters per second
   */
  public static double rawSensorVelocityToMPS(
      double rawSensorVelocity, double ticksPerRotation, double radius) {
    return rawSensorVelocity * (1 / 2048) * (2 * Math.PI * radius) * 0.1;
  }
  /**
   * Converts rotational speed in rotations per minute to raw sensor units
   * @param rpm Rotational speed, in rotations per minute, to convert
   * @param ticksPerRotation Number of ticks counted by the sensor when the mechanism rotates once
   * @return Converted rotational speed, expressed in raw sensor velocity
   */
  public static double rpmToRawSensorVelocity(double rpm, double ticksPerRotation) {
    return rpm * ticksPerRotation * (1 / 600);
  }
  /**
   * Converts a length in inches to encoder ticks
   * @param inches Length, in inches, to convert
   * @param radius Radius of the rotating mechanism
   * @param gearRatioMultiplier Gear ratio of the mechanism (driver to driven)
   * @param ticksPerRotation Number of ticks counted by the sensor when the mechanism rotates once
   * @return Converted length, expressed in sensor ticks
   */
  public static double inchesToTicks(
      double inches, double radius, double gearRatioMultiplier, double ticksPerRotation) {
    return (ticksPerRotation * inches * gearRatioMultiplier) / (2 * Math.PI * radius);
  }
  /**
   * Converts a length in encoder ticks to inches
   * @param ticks Length, in sensor ticks, to convert
   * @param radius RAdius of the rotating mechanism
   * @param gearRatioMultiplier Gear ratio of the mechanism (driver to driven)
   * @param ticksPerRotation Number of ticks counted by the sensor when the mechanism rotates once
   * @return Converted length, expressed in inches
   */
  public static double ticksToInches(
      double ticks, double radius, double gearRatioMultiplier, double ticksPerRotation) {
    return (2 * Math.PI * radius * ticks) / (ticksPerRotation * gearRatioMultiplier);
  }
  /**
   * Converts an angle in degrees to length in inches
   * @param theta Angle, in degrees, to convert
   * @param radius Radius of the rotating mechanism
   * @return Converted length, expressed in inches
   */
  public static double thetaToInches(double theta, double radius) {
    return theta * (Math.PI / 180) * radius;
  }
}
