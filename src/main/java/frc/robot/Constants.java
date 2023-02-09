// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import org.photonvision.SimVisionTarget;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static enum CanIds {
    // Westcoast drivetrain CAN IDs
    leftFalcon1(5),
    rightFalcon1(2),
    leftFalcon2(4),
    rightFalcon2(3),

    // Arm CAN IDs
    armMain1(12),
    armMain2(14),
    armExtended(51),

    // Gyro CAN IDs
    pigeon(0),
    extendedPigeon(51),
    ;

    public final int id;

    private CanIds(int id) {
      this.id = id;
    }
  }

  public static enum SensorIds {

    // Beambreak DIO IDs
    beambreak(2),

    // Limit switch DIO IDs
    limitswitch(0),
    ;

    public final int id;

    private SensorIds(int id) {
      this.id = id;
    }
  }

  public static final Transform3d kCameraToRobot = new Transform3d(
      new Translation3d(-0.25, 0, -.25), // in meters
      new Rotation3d());

  public static final int kGyroPin = 0;

  // IMPORTANT: This block of code is from 2020 game and is for testing purposes
  // only, we will update them to the 2023 season game
  public static final double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
  public static final double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
  public static final double kFarTgtXPos = Units.feetToMeters(54);
  public static final double kFarTgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75)
      - Units.inchesToMeters(48.0 / 2.0);
  public static final double kFarTgtZPos = (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

  public static final Pose3d kFarTargetPose = new Pose3d(
      new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
      new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));

  public static final SimVisionTarget kFarTarget = new SimVisionTarget(kFarTargetPose, targetWidth, targetHeight, 42);
  public static final DifferentialDriveKinematics kDtKinematics = new DifferentialDriveKinematics(
      RobotConstants.kTrackWidth);

  public static class VisionConstants {
    public static final String name1 = "terima";
    public static final String name2 = "teripaapa";
    public static final double kTargetHeight = 1.071626; // meters
    public static final double kCameraHeight = 0.5334; // meters
    public static final double mountingAngle = 42;
    public static final double focalLength = 2.9272781257541;
    public static final double camPitch = 0; // radians - ARBITRARY
    public static final double maxLEDRange = 20; // meters - ARBITRARY
    public static final double minTargetArea = 10; // square pixels - ARBITRARY
    public static final double trackingSpeed = 90; // fps
    public static final int camResWidth = 320; // pixels
    public static final int camResHeight = 240; // pixels
  }

  public static class RobotConstants {
    public static final double TalonFXTicksPerRotation = 2048;

    public static final double kTrackWidth = 0.6858; // meters

    public static final double kWheelRadius = 3 * 0.0254; // inches TO centimeters conversion
    public static final double kWheelCircumference = 2 * Math.PI * Constants.RobotConstants.kWheelRadius;

    public static final double timeStep = 0.02;

    public static final double mainArmGearRatio = 100;
  }

  public static class PowerConstants {
    // drive

    public static final double DriveBaseStraight = .55;
    public static final double DriveBaseTurn = .35;
    public static final double FeederVoltage = 11 * 0.9;;

    // intake
    public static final double intakeMultiplier = 1.0;
  }

  public static class ArmConstants {
    public static final double mainArmSetpoint1 = 13000;
    public static final double mainArmSetpoint2 = 18000;
  }

  public static class PIDConstants {
    // drive
    public static final double DriveBaseMotionMagickP = 0.5;
    public static final double DriveBaseMotionMagickI = 0;
    public static final double DriveBaseMotionMagickD = 0;

    // elevator
    public static final double ElevatorKp = 0.0035;
    public static final double ElevatorKf = -0.20459;

    // arm
    public static final double MainArmKp = 0.001;
    public static final double MainArmKi = 0;
    public static final double MainArmKd = 0;

    // extended arm
    public static final double ExtendedArmKp = 0.0001;
    public static final double ExtendedArmKi = 0.2;
    public static final double ExtendedArmKd = 0.2;
    public static final double ExtendedArmKTolerance = 0;
  }
};
