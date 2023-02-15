// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants.State;


public class RunGripper extends CommandBase {
  
  /** Creates a new RunGripper. */
  private State mode;
  private GripperSubsystem gripperSubsystem;
  private double power;

  /**
   * 
   * @param gripperSubsystem for controlling the power of the gripper.
   * @param mode to determine what mode the gripper should be in (intake or scoring).
   * @param power the power set to the gripper.
   */
  public RunGripper(GripperSubsystem gripperSubsystem, State mode, double power) {
    this.gripperSubsystem = gripperSubsystem;
    this.mode = mode;
    this.power = power;

    addRequirements(gripperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mode == State.INTAKE) {
      gripperSubsystem.setPower(power);
    }
    else if(mode == State.SCORE) {
      gripperSubsystem.setPower(-power);
    }
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
