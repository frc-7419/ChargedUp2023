// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveElevatorWithJoystick extends CommandBase {
  /** Creates a new MoveElevatorWithJoystick. */
  private ElevatorSubsystem elevatorSubsystem;
  private XboxController joystick;
  public MoveElevatorWithJoystick(ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getRightTriggerAxis()!=0){
      elevatorSubsystem.setPower(0.5);
    } else if (joystick.getLeftTriggerAxis()!=0){
      elevatorSubsystem.setPower(-0.5);
    } else {
      elevatorSubsystem.setPower(0);
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
