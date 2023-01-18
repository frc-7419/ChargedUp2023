// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunElevatorWithJoystick extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private XboxController joystick;

  public RunElevatorWithJoystick(ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.coast();
  }

  @Override
  public void execute() {
    if (joystick.getLeftY() != 0) {
      elevatorSubsystem.coast();
      elevatorSubsystem.setPower(joystick.getLeftY() * 0.5);
    } else {
      elevatorSubsystem.setPower(0);
      elevatorSubsystem.brake();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
