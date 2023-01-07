// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BrakeArms extends CommandBase {
  private ArmsSubsystem armsSubsystem;
  public BrakeArms(ArmsSubsystem armsSubsystem) {
    this.armsSubsystem = armsSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.armsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putBoolean("Arm Brake Cmd", true);
    armsSubsystem.brake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putBoolean("Arm Brake Cmd", true);
    armsSubsystem.brake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.putBoolean("Arm Brake Cmd", false);
    armsSubsystem.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
