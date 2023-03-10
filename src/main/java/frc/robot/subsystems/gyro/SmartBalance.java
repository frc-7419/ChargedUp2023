// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.BalanceConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class SmartBalance extends CommandBase {
  private GyroSubsystem gyroSubsystem;
  private DriveBaseSubsystem driveBaseSubsystem;
  private double robotPitch;
  private double lastTimeStamp;
  boolean stopCorrecting = false;

  /** Creates a new SmartBalance. */
  public SmartBalance(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    addRequirements(gyroSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTimeStamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTimeStamp = Timer.getFPGATimestamp();
    double timePassed = currentTimeStamp - lastTimeStamp;
    double previousPitch = robotPitch;
    robotPitch = gyroSubsystem.getPitch();
    double changeInPitch = (robotPitch - previousPitch) / (timePassed);
    if(!stopCorrecting) {
      if(Math.abs(robotPitch) < BalanceConstants.pitchTolerance) {
        driveBaseSubsystem.setAllPower(0);
        driveBaseSubsystem.brake(); 
        stopCorrecting = true;
      }
      else if(changeInPitch > BalanceConstants.pitchDerivativeThreshold) {
          driveBaseSubsystem.setAllPower(BalanceConstants.slowPower);
      }
      else { 
        driveBaseSubsystem.setAllPower(BalanceConstants.normalPower);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}