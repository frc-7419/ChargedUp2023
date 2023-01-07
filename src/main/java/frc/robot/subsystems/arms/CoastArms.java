// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CoastArms extends CommandBase {
  private ArmsSubsystem armsSubsystem;
  public CoastArms(ArmsSubsystem armsSubsystem) {
    this.armsSubsystem = armsSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.armsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armsSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armsSubsystem.coast();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armsSubsystem.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
