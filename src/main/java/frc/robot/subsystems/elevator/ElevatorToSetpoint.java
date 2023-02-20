// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorToSetpoint extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private double setpoint;

  public ElevatorToSetpoint(ElevatorSubsystem elevatorSubsystem, double setpoint) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.setpoint = setpoint;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setGoal(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setPower(0);
    elevatorSubsystem.brake();
  }

}
