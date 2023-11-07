// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.state;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.NodeConstants;

public class SwitchState extends CommandBase {
  /** Creates a new SwitchState. */
  private StateMachine stateMachine;
  private XboxController joystick;

  public SwitchState(StateMachine stateMachine, XboxController joystick) {
    this.joystick = joystick;
    this.stateMachine = stateMachine;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(stateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getPOV() == 0) {
      stateMachine.setPieceState(NodeConstants.PieceState.CONE);
    } else if (joystick.getPOV() == 1) {
      stateMachine.setPieceState(NodeConstants.PieceState.CUBE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
