// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import frc.robot.Constants.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class SmartBalanceNew extends CommandBase {
  /** Creates a new SmartBalanceNew. */
  private DriveBaseSubsystem driveBaseSubsystem;
  private GyroSubsystem gyroSubsystem;
  private PIDController angleController;
  private PIDController speedController;

  public SmartBalanceNew(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    addRequirements(driveBaseSubsystem, gyroSubsystem);
  }

  /**
   * Initialize the angle and speed PID controllers
   */
  @Override
  public void initialize() {
    angleController = new PIDController(
      PIDConstants.BalanceAngleKp, 
      PIDConstants.BalanceAngleKi, 
      PIDConstants.BalanceAngleKd);
    speedController = new PIDController(
      PIDConstants.BalanceSpeedKp, 
      PIDConstants.BalanceSpeedKi, 
      PIDConstants.BalanceSpeedKd);
    angleController.setSetpoint(0);
    speedController.setSetpoint(PIDConstants.BalanceSpeed);
    angleController.setTolerance(PIDConstants.BalanceAngleKTolerance);
    speedController.setTolerance(PIDConstants.BalanceSpeedKTolerance);
  }

  /**
   * Calculate the PID output based on the encoder velocity reading and the gyroscope pitch measurements.
   * Set the drivetrain power based on PID output
   */
  @Override
  public void execute() {
    double pitch = gyroSubsystem.getPitch();
    double calculatedDirection = angleController.calculate(pitch);

    double speed = driveBaseSubsystem.getLeftVelocity();
    double calculatedSpeed = angleController.calculate(speed);

    double calculatedOutput = Math.copySign(calculatedSpeed, calculatedDirection);
    
    // Setting the power
    driveBaseSubsystem.setAllPower(calculatedOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setAllPower(0);
    driveBaseSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleController.atSetpoint();
  }
}
