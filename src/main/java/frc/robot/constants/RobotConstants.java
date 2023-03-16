package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;

public final class RobotConstants {
  public static final double TalonFXTicksPerRotation = 2048;

  public static final double kTrackWidth = 0.6858; // meters

  public static final double kWheelRadius = 3 * 0.0254; // inches TO centimeters conversion
  public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;
  public static final String currentAlliance = Robot.getAllianceColor();

  public static final double mainArmGearRatio = 100;
  // arbitrary until we mount camera
  public static final Transform3d kCameraToRobot = new Transform3d();

  public static Transform3d kScoringCameraToRobot;

  public static Transform3d kAuxiliaryCameraToRobot;
}
