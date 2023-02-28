package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  public static final double driveStraight = 1;
  public static final double driveTurn = 1;
  public static final double driveTrainGearRatio = (double) (50.0 / 14) * (48.0 / 16);
  public static final double wheelDiameter = Units.inchesToMeters(6);
  public static final double wheelCircumference = Math.PI * wheelDiameter;
  public static final double unitsPerMeter = ((2048 * driveTrainGearRatio) / wheelCircumference);
  public static final double trackWidth = .68678;
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
  public static final double ks = 0.39188;
  public static final double kv = 2.5297;
  public static final double ka = 0.21837;
  public static final DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(trackWidth);
  public static final double kPDriveVelocity = 0.37841;
  public static final double maxVelocity = Units.feetToMeters(20);
  public static final double maxAcceleration = Units.feetToMeters(3);
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(trackWidth);
}
