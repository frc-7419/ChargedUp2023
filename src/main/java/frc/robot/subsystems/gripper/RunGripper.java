// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.GripperConstants.GripperState;

public class RunGripper extends CommandBase {

  /** Creates a new RunGripper. */
  private GripperState mode;

  private GripperSubsystem gripperSubsystem;

  /**
   * @param gripperSubsystem for controlling the power of the gripper.
   * @param mode to determine what mode the gripper should be in (intake or scoring).
   */
  public RunGripper(GripperSubsystem gripperSubsystem, GripperState mode) {
    this.gripperSubsystem = gripperSubsystem;
    this.mode = mode;

    addRequirements(gripperSubsystem);
  }

  // Adjust direction based on if robot is intaking or scoring
  @Override
  public void initialize() {
    if (mode == GripperState.INTAKE) {
      gripperSubsystem.coast();
      gripperSubsystem.setIntakePower(1);
    } else if (mode == GripperState.SCORE) {
      gripperSubsystem.coast();
      gripperSubsystem.setOuttakePower(0.5);
    } else if (mode == GripperState.HOLD) {
      gripperSubsystem.setIntakePower(0.1);
      gripperSubsystem.brake();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
