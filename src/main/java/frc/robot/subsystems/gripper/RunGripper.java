// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.GripperConstants;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.subsystems.state.StateMachine;

public class RunGripper extends CommandBase {

  /** Creates a new RunGripper. */
  private GripperState mode;

  private GripperSubsystem gripperSubsystem;
  private StateMachine stateMachine;

  /**
   * @param gripperSubsystem for controlling the power of the gripper.
   * @param mode             to determine what mode the gripper should be in
   *                         (intake or scoring).
   */
  public RunGripper(GripperSubsystem gripperSubsystem, GripperState mode, StateMachine stateMachine) {
    this.gripperSubsystem = gripperSubsystem;
    this.mode = mode;
    this.stateMachine = stateMachine;
    addRequirements(gripperSubsystem, stateMachine);
  }

  // Adjust direction based on if robot is intaking or scoring
  @Override
  public void initialize() {
    if (mode == GripperState.INTAKE_CUBE || mode == GripperState.SCORE_CONE) {
      gripperSubsystem.coast();
      gripperSubsystem.setIntakePower(GripperConstants.gripperPower);
      stateMachine.setIsHolding(false);
    } else if (mode == GripperState.INTAKE_CONE || mode == GripperState.SCORE_CUBE) {
      gripperSubsystem.coast();
      gripperSubsystem.setOuttakePower(GripperConstants.gripperPower);
      stateMachine.setIsHolding(false);
    } else if (mode == GripperState.HOLD) {
      gripperSubsystem.setIntakePower(0.1);
      gripperSubsystem.brake();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

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
