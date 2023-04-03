package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  public static final double driveStraight = 0.85;
  public static final double driveTurn = 0.7;

  public static final double slowStraight = 0.2;
  public static final double slowTurn = 0.2;
  ;
  public static final double driveTrainGearRatio = (double) (50.0 / 14) * (48.0 / 16);
  public static final double wheelDiameter = Units.inchesToMeters(6);
  public static final double wheelCircumference = Math.PI * wheelDiameter;
  public static final double unitsPerMeter = ((2048 * driveTrainGearRatio) / wheelCircumference);
  public static final double trackWidth = 0.55582;
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
  public static final double ks = 0.22983;
  public static final double kv = 2.5026;
  public static final double ka = 0.74527;
  public static final DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(trackWidth);
  public static final double kPDriveVelocity = 2.9766;
  public static final double kIDriveVelocity = 0;
  public static final double kDDriveVelocity = 0;
  public static final double maxVelocity = Units.feetToMeters(20);
  public static final double maxAcceleration = Units.feetToMeters(3);
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(trackWidth);
}
