// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.photonvision.SimVisionTarget;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static enum CanIds {
    // 2020 drive train ids
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

  public static class AprilTagPositionConstants {
    public static final Pose3d kAprilTagOnePose =
        new Pose3d(
            15.513558, 1.071626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(180)));

    public static final Pose3d kAprilTagTwoPose =
        new Pose3d(
            15.513558, 2.748026, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(180)));

    public static final Pose3d kAprilTagThreePose =
        new Pose3d(
            15.513558, 3.738626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(180)));

    public static final Pose3d kAprilTagFourPose =
        new Pose3d(
            16.178784,
            6.749796,
            0.695452,
            new Rotation3d(0, 0.695452, Units.degreesToRadians(180)));

    public static final Pose3d kAprilTagFivePose =
        new Pose3d(
            0.36195, 6.749796, 0.695452, new Rotation3d(0, 0.695452, Units.degreesToRadians(0)));

    public static final Pose3d kAprilTagSixPose =
        new Pose3d(1.02743, 3.738626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(0)));

    public static final Pose3d kAprilTagSevenPose =
        new Pose3d(1.02743, 2.748026, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(0)));

    public static final Pose3d kAprilTagEightPose =
        new Pose3d(1.02743, 1.071626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(0)));
  }

  public static class WaypointPositionConstants {
    public static final Translation2d kStartPose = new Translation2d(0, 0);
    // single substation and portal initial location constants

    // red mid waypoints

    public static final double kRedMidFirstWayPointX = 10.67;

    public static final double kRedMidFirstWayPointY = 2.15;

    public static final double kRedMidSecondWayPointX = 11.51;

    public static final double kRedMidSecondWayPointY = 0.86;

    public static final double kRedMidThirdWayPointX = 12.99;

    public static final double kRedMidThirdWayPointY = 0.57;

    // red single substation waypoints

    public static final double kRedSubstationFirstWayPointX = 7.70;

    public static final double kRedSubstationFirstWayPointY = 6.60;

    public static final double kRedSubstationSecondWayPointX = 5.66;

    public static final double kRedSubstationSecondWayPointY = 6.60;

    // blue mid waypoints

    public static final double kBlueMidFirstWayPointX = 6.07;

    public static final double kBlueMidFirstWayPointY = 3.31;

    public static final double kBlueMidSecondWayPointX = 5.49;

    public static final double kBlueMidSecondWayPointY = 4.48;

    public static final double kBlueMidThirdWayPointX = 3.91;

    public static final double kBlueMidThirdWayPointY = 4.76;

    // blue single substation waypoints

    public static final double kBlueSubstationFirstWayPointX = 8.38;

    public static final double kBlueSubstationFirstWayPointY = 6.41;

    public static final double kBlueSubstationSecondWayPointX = 10.34;

    public static final double kBlueSubstationSecondWayPointY = 6.72;

    // waypoint angles

    public static final double kHeadingFront = 0; // degrees

    public static final double kHeadingPerpendicular = 90; // degrees

    public static final double kHeadingPerpendicularReverse = -90; // degrees

    public static final double kHeadingAvoidChargeStation = 135; // degrees

    public static final double kHeadingAvoidChargeStationReverse = -45; // degrees

    public static final double kHeadingReverse = 180; // degrees

    // constructing waypoints

    // red waypoints

    // red mid waypoints

    public static final PathPoint kRedMidFirstWayPoint =
        new PathPoint(
            new Translation2d(kRedMidFirstWayPointX, kRedMidFirstWayPointY),
            Rotation2d.fromDegrees(kHeadingFront));

    public static final PathPoint kRedMidSecondWayPoint =
        new PathPoint(
            new Translation2d(kRedMidSecondWayPointX, kRedMidSecondWayPointY),
            Rotation2d.fromDegrees(kHeadingPerpendicularReverse));

    public static final PathPoint kRedMidThirdWayPoint =
        new PathPoint(
            new Translation2d(kRedMidThirdWayPointX, kRedMidThirdWayPointY),
            Rotation2d.fromDegrees(kHeadingFront));

    // red single substation waypoints

    public static final PathPoint kRedSubstationFirstWayPoint =
        new PathPoint(
            new Translation2d(kRedSubstationFirstWayPointX, kRedSubstationFirstWayPointY),
            Rotation2d.fromDegrees(kHeadingReverse));

    public static final PathPoint kRedSubstationSecondWayPoint =
        new PathPoint(
            new Translation2d(kRedSubstationSecondWayPointX, kRedSubstationSecondWayPointY),
            Rotation2d.fromDegrees(kHeadingReverse));

    // blue waypoints

    // blue mid waypoints

    public static final PathPoint kBlueMidFirstWayPoint =
        new PathPoint(
            new Translation2d(kBlueMidFirstWayPointX, kBlueMidFirstWayPointY),
            Rotation2d.fromDegrees(kHeadingPerpendicular));

    public static final PathPoint kBlueMidSecondWayPoint =
        new PathPoint(
            new Translation2d(kBlueMidSecondWayPointX, kBlueMidSecondWayPointY),
            Rotation2d.fromDegrees(kHeadingAvoidChargeStation));

    public static final PathPoint kBlueMidThirdWayPoint =
        new PathPoint(
            new Translation2d(kBlueMidThirdWayPointX, kBlueMidThirdWayPointY),
            Rotation2d.fromDegrees(kHeadingReverse));

    // blue single substation waypoints

    public static final PathPoint kBlueSubstationFirstWayPoint =
        new PathPoint(
            new Translation2d(kBlueSubstationFirstWayPointX, kBlueSubstationFirstWayPointY),
            Rotation2d.fromDegrees(kHeadingFront));

    public static final PathPoint kBlueSubstationSecondWayPoint =
        new PathPoint(
            new Translation2d(kBlueSubstationSecondWayPointX, kBlueSubstationSecondWayPointY),
            Rotation2d.fromDegrees(kHeadingFront));
  }

  public static class RobotConstants {
    public static final Alliance currentAlliance = Robot.getAllianceColor();

    public static final double TalonFXTicksPerRotation = 2048;

    public static final double kTrackWidth = 0.6858; // meters

    public static final double kWheelRadius = 3 * 0.0254; // inches TO centimeters conversion
    public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;

    public static final double timeStep = 0.02;

    public static final double mainArmGearRatio = 100;

    public static final Transform3d kCameraToRobot =
        new Transform3d(
            new Translation3d(-0.25, 0, -.25), // in meters
            new Rotation3d());

    public static final int kGyroPin = 0;

    // IMPORTANT: This block of code is from 2020 game and is for testing purposes
    // only, we will update them to the 2023 season game
    public static final double targetWidth =
        Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    public static final double targetHeight =
        Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    public static final double kFarTgtXPos = Units.feetToMeters(54);
    public static final double kFarTgtYPos =
        Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    public static final double kFarTgtZPos =
        (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

    public static final Pose3d kFarTargetPose =
        new Pose3d(
            new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));

    public static final SimVisionTarget kFarTarget =
        new SimVisionTarget(kFarTargetPose, targetWidth, targetHeight, 42);
  }

  public static class VisionConstants {
    public static final String name1 = "terima";
    public static final String name2 = "teripaapa";
    public static final double kTargetHeight = 1.071626; // meters
    public static final double kCameraHeight = 0.5334; // meters
    public static final double kCameraPitch = 42;
    public static final double visionAmbiguityThreshold = 0.2;
    public static final double focalLength = 2.9272781257541;
    public static final int cameraResolutionWidth = 320; // pixels
    public static final int cameraResolutionHeight = 240; // pixels
  }

  public static class ArmConstants {
    public static final double mainArmSetpoint1 = 13000;
    public static final double mainArmSetpoint2 = 18000;
    public static final double mainArmPowerCoefficient = 0.5;
    public static final double extendedArmPowerCoefficient = 0.3;
  }

  public static class DriveConstants {
    public static final double driveTrainGearRatio = (50 / 14) * (48 / 16);
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

    // smart balance
    public static final double BalanceAngleKp = 0.005;
    public static final double BalanceAngleKi = 0;
    public static final double BalanceAngleKd = 0;
    public static final double BalanceAngleKTolerance = 2;
  }
}
;
