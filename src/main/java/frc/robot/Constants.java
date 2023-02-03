// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.SimVisionTarget;

import com.pathplanner.lib.PathPoint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static enum CanIds {
        // 2020 drive train ids
        leftFalcon1(5),
        rightFalcon1(2),
        leftFalcon2(4),
        rightFalcon2(3),
        intakeSpark(32),
        loaderVictor(16),
        feederTalon(23),
        armSpark1(11),
        armSpark2(12),
        rightElevatorFalcon(50),
        leftElevatorFalcon(51),
        pigeon(0)
        ;

        public final int id;

        private CanIds(int id) {
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
        public static final double visionAmbiguityThreshold = 0.2;
        public static final double focalLength = 2.9272781257541;
        public static final double camPitch = 0; // radians - ARBITRARY
        public static final double maxLEDRange = 20; // meters - ARBITRARY
        public static final double minTargetArea = 10; // square pixels - ARBITRARY
        public static final double trackingSpeed = 90; // fps
        public static final int camResWidth = 320; // pixels
        public static final int camResHeight = 240; // pixels

    }

    public static class WaypointConstants {
        public static final Translation2d kStartPose = new Translation2d(0, 0);
        // single substation and portal initial location constants
        
        public static final double kBlueX = 12.58;
        public static final double kBlueY = 0.91;
        public static final double kRedX = 6.87;
        public static final double kRedY = 6.57;
        public static final double kHeadingBlue = 0; //degrees
        public static final double kHeadingRed = 180; //degrees
        public static final PathPoint kBluePoint2 = new PathPoint(new Translation2d(kBlueX, kBlueY), Rotation2d.fromDegrees(kHeadingBlue));
        public static final PathPoint kBluePoint1 = new PathPoint(new Translation2d(12.3, 0.85), Rotation2d.fromDegrees(kHeadingBlue));
        public static final PathPoint kRedPoint = new PathPoint(new Translation2d(kRedX, kRedY), Rotation2d.fromDegrees(kHeadingRed));
        public static final PathPoint kAlignWithRight = new PathPoint(new Translation2d(3.59, 0.63), Rotation2d.fromDegrees(180));
        public static final PathPoint kAlignWithLeft = new PathPoint(new Translation2d(3.59, 0.63), Rotation2d.fromDegrees(180));
    }

    public static class RobotConstants {
        public static final double TalonFXTicksPerRotation = 2048;

        public static final double kTrackWidth = 0.6858; // meters

        public static final double kWheelRadius = 3 * 0.0254; // inches TO centimeters conversion
        public static final double kWheelCircumference = 2 * Math.PI * Constants.RobotConstants.kWheelRadius;

        public static final double timeStep = 0.02;
    }

    public static class PowerConstants {
        // drive

        public static final double DriveBaseStraight = .55;
        public static final double DriveBaseTurn = .35;
        public static final double FeederVoltage = 11 * 0.9;;

        // intake
        public static final double intakeMultiplier = 1.0;
    }
    public static class DriveConstants {
        public static final double gearRatio = (50/14)*(48/16);
        public static final double wheelDiameter = Units.inchesToMeters(6);
        public static final double wheelCircumference = Math.PI*wheelDiameter;
        public static final double unitsPerMeter = ((2048*gearRatio)/wheelCircumference);
        public static final double trackWidth = .68678; 
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        // public static final double ks = 0.075829;
        // public static final double kv = 2.4732; 
        // public static final double ka = 0.25441;
        public static final double ks = 0.39188;
        public static final double kv = 2.5297;
        public static final double ka = 0.21837;
        public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackWidth);
        public static final double kPDriveVelocity = 0.37841;
        public static final double maxVelocity = Units.feetToMeters(20);
        public static final double maxAcceleration = Units.feetToMeters(3);
    }
    public static class PIDConstants {
        // drive
        public static final double DriveBaseMotionMagickP = 0.5;
        public static final double DriveBaseMotionMagickI = 0;
        public static final double DriveBaseMotionMagickD = 0;

        // elevator
        public static final double ElevatorKp = 0.0035;
        public static final double ElevatorKf = -0.20459;
    }

};