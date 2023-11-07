// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.GripperConstants;
import frc.robot.constants.NodeConstants;
import frc.robot.subsystems.state.StateMachine;

public class AutoRunGripper extends CommandBase {
  /** Creates a new SmartRunGripper. */
  private GripperSubsystem gripperSubsystem;
  private StateMachine stateMachine;
  private boolean checkingHold;
  private double lastTimeStamp;

  public AutoRunGripper(
      GripperSubsystem gripperSubsystem, StateMachine stateMachine) {
    this.gripperSubsystem = gripperSubsystem;
    this.stateMachine = stateMachine;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripperSubsystem, stateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateMachine.setIsHolding(false);
    gripperSubsystem.coast();
    checkingHold = false;
    lastTimeStamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stateMachine.getPieceState() == NodeConstants.PieceState.CUBE) {
      gripperSubsystem.setIntakePower(GripperConstants.gripperPower);
    } else{
      gripperSubsystem.setIntakePower(-GripperConstants.gripperPower);
    }
    double timePassed = Timer.getFPGATimestamp() - lastTimeStamp;
    if (timePassed > GripperConstants.gripperDelaySeconds) {
      checkingHold = true;
    }
    if (checkingHold) {
      if (Math.abs(gripperSubsystem.getVelocity()) < GripperConstants.stallVelocityThreshold) {
        stateMachine.setIsHolding(true);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSubsystem.setPower(0);
    gripperSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean holdMode = stateMachine.getHoldingState();
    return holdMode;
  }
}
