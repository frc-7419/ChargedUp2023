// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunElevatorWithJoystick extends CommandBase {
  /** Creates a new RunElevatorWithJoystick. */
  private ElevatorSubsystem elevatorSubsystem;

  private XboxController joystick;

  public RunElevatorWithJoystick(ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.brake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getYButton()) {
      elevatorSubsystem.setPower(0.5);
    } else if (joystick.getAButton()) {
      elevatorSubsystem.setPower(-0.5);
    } else {
      elevatorSubsystem.setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setPower(0);
    elevatorSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
