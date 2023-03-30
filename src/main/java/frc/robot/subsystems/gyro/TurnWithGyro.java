// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.GyroConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class TurnWithGyro extends CommandBase {
  /** Creates a new TurnWithGyro. */
  private DriveBaseSubsystem driveBaseSubsystem;
  private GyroSubsystem gyroSubsystem;
  private double desiredAngle;
  private double tolerance;
  private PIDController pidController;
  public TurnWithGyro(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, double desiredAngle, double tolerance) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    this.desiredAngle = desiredAngle;
    this.tolerance = tolerance;
    addRequirements(driveBaseSubsystem, gyroSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
    pidController = new PIDController(GyroConstants.kp, GyroConstants.ki, GyroConstants.kd);
    double currentAngle = gyroSubsystem.getYaw();
    double deltaTheta = desiredAngle - currentAngle;
    pidController.setSetpoint(deltaTheta);
    pidController.setTolerance(tolerance);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(gyroSubsystem.getYaw());
    driveBaseSubsystem.setLeftPower(output);
    driveBaseSubsystem.setRightPower(-output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.brake();
    driveBaseSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
