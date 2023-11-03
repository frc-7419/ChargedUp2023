// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunDrive extends CommandBase {
  /** Creates a new RunDrive. */
  private double leftPower;
  private double rightPower;

  private DriveBaseSubsystem driveBaseSubsystem;

  public RunDrive(DriveBaseSubsystem driveBaseSubsystem, double power) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.leftPower = power;
    this.rightPower = power;
  }
  public RunDrive(DriveBaseSubsystem driveBaseSubsystem, double leftPower, double rightPower) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBaseSubsystem.setLeftPower(leftPower);
    driveBaseSubsystem.setLeftPower(rightPower);
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
    return false;
  }
}
