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
        topShooterFalcon(10), 
        bottomShooterFalcon(14),
        intakeSpark(32),  
        loaderVictor(16),
        turretFalcon(62),
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

        public static final double BottomShooterWheelRadius = 0.0508; // meters
        public static final double TopShooterWheelRadius = 0.0508;

        public static final double RotationsPerMeter = 1/(2*Math.PI*0.0508);

        // top shooter kS, kV, kA
        public static final double TopShooterKs = 0.56452;
        public static final double TopShooterKv = 0.11144;
        public static final double TopShooterKa = 0.026061;

        // bottom shooter kS, kV, kA, meters
        public static final double BottomShooterKs = 0.67661;
        public static final double BottomShooterKv = 0.10994;
        public static final double BottomShooterKa = 0.0060832;;

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

        // turn with gyro gains:
        public static final double GyrokP32 = 0.0085;
        public static final double GyrokI32 = 0.00;
        public static final double GyrokD32 = 0.0001;

        public static final double GyrokP45 = 0.007;
        public static final double GyrokI45 = 0;
        public static final double GyrokD45 = 0.0001;

        public static final double GyrokP63 = 0.007;
        public static final double GyrokI63 = 0.0002;
        public static final double GyrokD63 = 0.0003;

        public static final double GyrokP80 = 0.0064;
        public static final double GyrokI80 = 0.000155;
        public static final double GyrokD80 = 0.00025;

        public static final double GyrokP85 = 0.0064;
        public static final double GyrokI85 = 0.000155;
        public static final double GyrokD85 = 0.00025;

        public static final double GyrokP90 = 0.0064;
        public static final double GyrokI90 = 0.000155;
        public static final double GyrokD90 = 0.00025;

        public static final double GyrokP115 = 0.005;
        public static final double GyrokI115 = 0.0;
        public static final double GyrokD115 = 0.0000083;

        public static final double GyrokP122 = 0.0072;
        public static final double GyrokI122 = 0.00048;
        public static final double GyrokD122 = 0.00050;

        public static final double GyrokP135 = 0.0047;
        public static final double GyrokI135 = 0;
        public static final double GyrokD135 = 0.0000083;

        public static final double GyrokP180 = 0.0035;
        public static final double GyrokI180 = 0.000655;
        public static final double GyrokD180 = 0.0004;

        public static final double GyrokP185 = 0.0035;
        public static final double GyrokI185 = 0.000655;
        public static final double GyrokD185 = 0.0004;

        

        // turret PID gains
        // public static final double TurretKp = 0.0065;
        public static final double TurretKp = 0.015;
        public static final double TurretKi = 0.01;
        public static final double TurretKd = 0.0;

        //shooter, meters
        public static final double TopShooterkP = .0014651; //insert here
        public static final double TopShooterkI = 0;
        public static final double TopShooterkD = 0;

        public static final double BottomShooterkP = 0.00039888;
        public static final double BottomShooterkI = 0;
        public static final double BottomShooterkD = 0;

        //elevator
        public static final double ElevatorKp = 0.0035;
        // public static final double ElevatorKf = -0.10459;
        public static final double ElevatorKf = -0.20459;
    }

    public static final Double[][] kDistanceToTopShooterVelocity = {
        {0.738, 19.5},
        // {1.03, 31.5},
        {1.098, 19.5},
        {1.3, 26.0},
        {1.5, 25.0},
        {1.92, 28.5},
        {2.3, 30.0},
        {2.5, 34.6},
        {2.7, 36.5},
        {2.9, 38.0}
    };

    public static final Double[][] kDistanceToBottomShooterVelocity = {
        {0.738, 32.0},
        // {1.03, 31.5},
        {1.098, 32.0},
        {1.3, 30.0},
        {1.5, 32.0},
        {1.92, 32.0},
        {2.3, 35.0},
        {2.5, 37.0},
        {2.7, 37.0},
        {2.9, 39.0}
    };

    // public static final Double[][] kRawVelocityToTopFf = {
    //     {1000.0, 0.06365},
    //     {1500.0, 0.058615},
    //     {2000.0, 0.0555},
    //     {2500.0, 0.0535},
    //     {3000.0, 0.0525},
    //     {3500.0, 0.0506},
    //     {4000.0, 0.0495},
    //     {4500.0, 0.049425},
    //     {5000.0, 0.0488},
    //     {5500.0, 0.0491},
    //     {6000.0, 0.048305},
    //     {6500.0, 0.048305},
    //     {7000.0, 0.0486},
    //     {7500.0, 0.048875},
    //     {8000.0, 0.04874},
    //     {8500.0, 0.04876},
    //     {9000.0, 0.048},
    //     {10000.0, 0.04785},
    //     {10500.0, 0.04775},
    //     {11000.0, 0.04775},
    //     {12000.0, 0.049},
    //     {12500.0, 0.0491},
    //     {13000.0, 0.0484},
    //     {13500.0, 0.0525},
    //     {14000.0, 0.0545}
    // };

    // public static final Double[][] kRawVelocityToBottomFf = {
    //     {1000.0, 0.0615},
    //     {1500.0, 0.0552},
    //     {2000.0, 0.05},
    //     {2500.0, 0.04815},
    //     {3000.0, 0.04775},
    //     {3500.0, 0.04735},
    //     {4000.0, 0.047545},
    //     {4500.0, 0.0473445},
    //     {5000.0, 0.0471},
    //     {5500.0, 0.0475},
    //     {6000.0, 0.047385},
    //     {6500.0, 0.0477425},
    //     {7000.0, 0.0477425},
    //     {7500.0, 0.047761},
    //     {8000.0, 0.04772},
    //     {8500.0, 0.047827},
    //     {9000.0, 0.04735},
    //     {9500.0, 0.04725},
    //     {10000.0, 0.04725},
    //     {10500.0, 0.04725},
    //     {11000.0, 0.0472},
    //     {11500.0, 0.0472},
    //     {12000.0, 0.0485},
    //     {12500.0, 0.0486},
    //     {13000.0, 0.0481},
    //     {13500.0, 0.0515},
    //     {14000.0, 0.05}
    // };
};