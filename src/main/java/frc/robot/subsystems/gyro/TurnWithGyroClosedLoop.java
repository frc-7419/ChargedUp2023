package frc.robot.subsystems.gyro;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class TurnWithGyroClosedLoop extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private GyroSubsystem gyroSubsystem;
  private double target;
  private double tolerance;
  private double kP;
  private double kI;
  private double kD;
  private PIDController pidController;
  private double pidOutput;
  private double initAngle;

  /**
   * LEFT IS POSITIVE
   * 
   * @param driveBaseSubsystem
   * @param gyro
   * @param angle
   */
  public TurnWithGyroClosedLoop(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, double target,
      double tolerance, double kP, double kI, double kD) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    this.target = target;
    this.tolerance = tolerance;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
    initAngle = gyroSubsystem.getYaw();
    pidController = new PIDController(kP, kI, kD);
    pidController.setSetpoint(initAngle + target);
    pidController.setTolerance(tolerance);
  }

  @Override
  public void execute() {
    pidOutput = pidController.calculate(gyroSubsystem.getYaw());
    driveBaseSubsystem.setLeftPower(pidOutput);
    driveBaseSubsystem.setRightPower(-pidOutput);
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.stop();
    driveBaseSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
