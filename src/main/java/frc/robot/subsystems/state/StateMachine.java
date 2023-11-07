// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.state;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.NodeConstants;
import frc.robot.constants.ElevatorConstants.NodeState;

public class StateMachine extends SubsystemBase {
  /** Creates a new StateMachine. */
  private NodeConstants.PieceState pieceState;
  private boolean isHolding;

  public StateMachine() {
    this.pieceState = NodeConstants.PieceState.CONE;
    this.isHolding = false;
  }

  public void setPieceState(NodeConstants.PieceState pieceState) {
    this.pieceState = pieceState;
  }

  public NodeConstants.PieceState getPieceState() {
    return pieceState;
  }

  public void setIsHolding(boolean isHolding) {
    this.isHolding = isHolding;
  }

  public boolean getHoldingState() {
    return isHolding;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Piece State", pieceState + "");
    SmartDashboard.putBoolean("Holding State", isHolding);

  }
}
