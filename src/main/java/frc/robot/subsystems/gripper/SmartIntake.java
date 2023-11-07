// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.GripperConstants;
import frc.robot.constants.NodeConstants;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.state.StateMachine;

public class SmartIntake extends CommandBase {
  /** Creates a new SmartRunGripper. */
  private GripperSubsystem gripperSubsystem;
  private StateMachine stateMachine;
  private boolean checkingHold;
  private double lastTimeStamp;

  public SmartIntake(
      GripperSubsystem gripperSubsystem, StateMachine stateMachine, GripperState mode, LedSubsystem ledSubsystem) {
    this.gripperSubsystem = gripperSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripperSubsystem, stateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripperSubsystem.coast();
    lastTimeStamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stateMachine.getPieceState() == NodeConstants.PieceState.CUBE) {
      gripperSubsystem.setIntakePower(GripperConstants.gripperPower);
    } else if (stateMachine.getPieceState() == NodeConstants.PieceState.CONE) {
      gripperSubsystem.setIntakePower(-GripperConstants.gripperPower);
    }
    if (checkingHold) {
      if (gripperSubsystem.getVelocity() < GripperConstants.stallVelocityThreshold) {
        checkingHold = true;
      }
      double timePassed = Timer.getFPGATimestamp() - lastTimeStamp;
      if (timePassed > GripperConstants.gripperDelaySeconds) {
        checkingHold = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean holdMode = stateMachine.getHoldingState();
    return holdMode;
  }
}
