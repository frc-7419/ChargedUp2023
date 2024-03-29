package frc.robot.constants;

public final class PIDConstants {
  // drive
  public static final double DriveBaseMotionMagickP = 0.5;
  public static final double DriveBaseMotionMagickI = 0;
  public static final double DriveBaseMotionMagickD = 0;

  // elevator
  public static final double ElevatorKp = 0.0035;
  public static final double ElevatorKi = 0;
  public static final double ElevatorKd = 0;
  public static final double ElevatorKf = -0.20459;
  public static final double ElevatorKTolerance = 0.15;

  // arm
  public static final double armKp = 0.3;
  public static final double armKi = 0;
  public static final double armKd = 0;
  public static final double armTolerance = 0.15;

  // smart balance
  public static final double BalanceAngleKp = 0.01;
  public static final double BalanceAngleKi = 0;
  public static final double BalanceAngleKd = 0;
  public static final double BalanceAngleKTolerance = 2;
  public static final double BalanceSpeedKp = 0.1;
  public static final double BalanceSpeedKi = 0.0001;
  public static final double BalanceSpeedKd = 0;
  public static final double BalanceSpeed = 0.35; // desired robot speed in meter/s
  public static final double BalanceSpeedKTolerance = 0.005;
  public static final double BalanceSpeedkF = 0;

  // wrist PID
  public static final double wristkP = 0.032829;
  public static final double wristkI = 0;
  public static final double wristkD = 0;
  public static final double wristTolerance = 0.1;
}
