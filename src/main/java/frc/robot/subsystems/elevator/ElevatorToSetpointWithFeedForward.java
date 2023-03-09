// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.NodeState;

public class ElevatorToSetpointWithFeedForward extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private double setpoint;
  private TrapezoidProfile currentProfile;
  private ElevatorFeedforward feedforward;
  private PIDController elevatorPIDController;

  /** Creates a new ElevatorToSetpointWithFeedForward. */
  public ElevatorToSetpointWithFeedForward(
      ElevatorSubsystem elevatorSubsystem, NodeState nodeState) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.setpoint = nodeState.elevatorSetpoint;
    this.feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.elevatorKs,
            ElevatorConstants.elevatorKg,
            ElevatorConstants.elevatorKv,
            ElevatorConstants.elevatorKa);
    this.elevatorPIDController = new PIDController(0.3, 0, 0);
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setGoal(setpoint);
    elevatorSubsystem.setSetpoint(
        new TrapezoidProfile.State(elevatorSubsystem.getElevatorPosition(), 0));

    SmartDashboard.putNumber("Goal Position", elevatorSubsystem.getGoal().position);
    SmartDashboard.putNumber("Setpoint Position", elevatorSubsystem.getSetpoint().position);
  }

  // Called every time the schseduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentProfile =
        new TrapezoidProfile(
            elevatorSubsystem.getConstraints(),
            elevatorSubsystem.getGoal(),
            elevatorSubsystem.getSetpoint());

    double currentPosition = elevatorSubsystem.getElevatorPosition();

    TrapezoidProfile.State nextSetpoint = currentProfile.calculate(0.02);

    double feedForwardPower = feedforward.calculate(nextSetpoint.velocity);

    elevatorSubsystem.setSetpoint(nextSetpoint);

    SmartDashboard.putNumber("Trapezoid Position", nextSetpoint.position);

    elevatorPIDController.setSetpoint(nextSetpoint.position);

    double elevatorPower = elevatorPIDController.calculate(currentPosition);
    elevatorSubsystem.setPower(elevatorPower + feedForwardPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elevatorSubsystem.getSetpoint().position == elevatorSubsystem.getGoal().position);
  }
}
