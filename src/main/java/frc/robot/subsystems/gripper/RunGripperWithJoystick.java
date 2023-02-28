// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunGripperWithJoystick extends CommandBase {
  private GripperSubsystem gripperSubsystem;
  private XboxController joystick;

  public RunGripperWithJoystick(GripperSubsystem gripperSubsystem, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripperSubsystem = gripperSubsystem;
    this.joystick = joystick;
    addRequirements(gripperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripperSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getXButton()) {
      gripperSubsystem.setPower(-1);
    }
    else if (joystick.getBButton()) {
      gripperSubsystem.setPower(1);
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
