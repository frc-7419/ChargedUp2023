// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorWithMotionMagic extends CommandBase {
  ElevatorSubsystem elevatorSubsystem;
  private double setpoint;
  public ElevatorWithMotionMagic(ElevatorSubsystem elevatorSubsystem, double setpoint) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.setpoint = setpoint;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setMotionMagic(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("MM Position", elevatorSubsystem.getElevatorIntegratedPosition());
    SmartDashboard.putNumber("MM Output", elevatorSubsystem.getElevatorOutput());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
