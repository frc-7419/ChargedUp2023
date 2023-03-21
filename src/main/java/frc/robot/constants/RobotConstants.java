package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;

public final class RobotConstants {
  public static final double TalonFXTicksPerRotation = 2048;

  public static final double kTrackWidth = 0.6858; // meters

  public static final double kWheelRadius = 3 * 0.0254; // inches TO centimeters conversion
  public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;
  public static final String currentAllianceColor = Robot.getAllianceColor();
  public static final Alliance currentAlliance = Robot.getAlliance();
  public static final String currentAllianceSide = Robot.getAllianceSide();

  public static final double mainArmGearRatio = 100;
  // arbitrary until we mount camera
  public static final Transform3d kCameraToRobot = new Transform3d();

  public static double armElevatorDelay = 0.5;

  public static double joystickDeadZone = 0.07;

  public static double maxVoltage = 12;
}
