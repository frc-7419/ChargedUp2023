// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.constants.RobotConstants;

public class ElevatorToSetpointWithFeedForward extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private double desiredPosition;
  private TrapezoidProfile currentProfile;
  private ElevatorFeedforward feedforward;
  private PIDController elevatorPIDController;

  /** Creates a new ElevatorToSetpointWithFeedForward. */
  public ElevatorToSetpointWithFeedForward(
      ElevatorSubsystem elevatorSubsystem, NodeState nodeState) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.desiredPosition = nodeState.elevatorSetpoint;
    this.feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.elevatorKs,
            ElevatorConstants.elevatorKg,
            ElevatorConstants.elevatorKv,
            ElevatorConstants.elevatorKa);
    this.elevatorPIDController =
        new PIDController(
            ElevatorConstants.elevatorKp,
            ElevatorConstants.elevatorKi,
            ElevatorConstants.elevatorKd);
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setGoal(desiredPosition);
    elevatorSubsystem.setSetpoint(
        new TrapezoidProfile.State(elevatorSubsystem.getElevatorIntegratedPosition(), 0));

    SmartDashboard.putNumber("Elevator Goal Position", elevatorSubsystem.getGoal().position);
    SmartDashboard.putNumber(
        "Elevator Setpoint Position", elevatorSubsystem.getSetpoint().position);
  }

  // Called every time the schseduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentProfile =
        new TrapezoidProfile(
            ElevatorConstants.constraints,
            elevatorSubsystem.getGoal(),
            elevatorSubsystem.getSetpoint());

    double currentPosition = elevatorSubsystem.getElevatorIntegratedPosition();

    TrapezoidProfile.State nextSetpoint = currentProfile.calculate(0.02);

    double feedForwardVoltage = feedforward.calculate(nextSetpoint.velocity);

    double feedForwardPower = feedForwardVoltage / RobotConstants.maxVoltage;

    SmartDashboard.putNumber("Elevator Feedforward Power", feedForwardPower);

    SmartDashboard.putNumber("Elevator Current Position", currentPosition);

    SmartDashboard.putNumber("Elevator Predicted Position", nextSetpoint.position);

    elevatorSubsystem.setSetpoint(nextSetpoint);

    elevatorPIDController.setSetpoint(nextSetpoint.position);

    double elevatorPower = elevatorPIDController.calculate(currentPosition);

    SmartDashboard.putNumber("Elevator PID Power", elevatorPower);

    double pidFeedforwardPower = elevatorPower + feedForwardPower;

    elevatorSubsystem.setPower(pidFeedforwardPower);
    elevatorSubsystem.brake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // elevatorSubsystem.coast();
    elevatorSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    elevatorSubsystem.brake();
    double error =
        elevatorSubsystem.getGoal().position - elevatorSubsystem.getElevatorIntegratedPosition();
    boolean isAtSetpoint = Math.abs(error) <= 0.02;
    return isAtSetpoint;
  }
}
