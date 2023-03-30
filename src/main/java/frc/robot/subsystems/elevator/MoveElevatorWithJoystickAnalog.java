// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConstants;

public class MoveElevatorWithJoystickAnalog extends CommandBase {
  /** Creates a new MoveElevatorWithJoystick. */
  private ElevatorSubsystem elevatorSubsystem;

  private XboxController joystick;

  public MoveElevatorWithJoystickAnalog(
      ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
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
    double joystickElevatorPower = -joystick.getRightY() * ElevatorConstants.elevatorPower;
    if (Math.abs(joystickElevatorPower) > 0) {
      elevatorSubsystem.coast();
    } else {
      elevatorSubsystem.brake();
    }
    double elevatorFeedForward = -Math.copySign(ElevatorConstants.elevatorFeedForward, joystickElevatorPower);
    elevatorSubsystem.setPower(joystickElevatorPower + elevatorFeedForward);
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
