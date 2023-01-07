// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        ;
        
        public final int id;
        private CanIds(int id) {
            this.id = id;
        }
    }

    public static class LimelightConstants {
        public static final double kTargetHeight = 2.6416; //meters
        public static final double kCameraHeight = 1.07; // meters
        public static final double mountingAngle = 42;
        public static final double focalLength = 2.9272781257541;
    }

    public static class RobotConstants {
        public static final double TalonFXTicksPerRotation = 2048;

        public static final double RotationsPerMeter = 1/(2*Math.PI*0.0508);

        public static final double trackWidth = 0.69; // meters
    }

    public static class PowerConstants {
        //drive
        // public static final double DriveBaseLeftStraight = .45;
        // public static final double DriveBaseRightTurn = .35; //.6
        // public static final double DriveBaseLeftTurn = .35; //.6
        // public static final double DriveBaseRightStraight = .45;

        public static final double DriveBaseStraight = .55;
        public static final double DriveBaseTurn = .35;
        public static final double FeederVoltage = 11*0.9;
        ; 
        // public static final double DriveBaseLeftStraight = -.15;
        // public static final double DriveBaseRightTurn = .1; 
        // public static final double DriveBaseLeftTurn = .1; 
        // public static final double DriveBaseRightStraight = -.15;


        //intake
        public static final double intakeMultiplier = 1.0; 
    }

    public static class PIDConstants {
        //drive
        public static final double DriveBaseMotionMagickP = 0.5;
        public static final double DriveBaseMotionMagickI = 0;
        public static final double DriveBaseMotionMagickD = 0;

        //elevator
        public static final double ElevatorKp = 0.0035;
        // public static final double ElevatorKf = -0.10459;
        public static final double ElevatorKf = -0.20459;
    }

};