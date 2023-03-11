// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.constants.PIDConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class WristToSetpointWithFeedforward extends CommandBase {
  /** Creates a new ArmToSetpointWithFeedforward. */
  private WristSubsystem wristSubsystem;

  private ArmSubsystem armSubsystem;

  private double setpoint;
  private TrapezoidProfile currentProfile;
  private ArmFeedforward feedforward;
  private PIDController wristPIDController;

  public WristToSetpointWithFeedforward(
      WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, NodeState wristState) {
    this.wristSubsystem = wristSubsystem;
    this.armSubsystem = armSubsystem;
    this.setpoint = wristState.wristSetpoint;
    this.feedforward =
        new ArmFeedforward(
            WristConstants.ks, WristConstants.kg, WristConstants.kv, WristConstants.ka);
    this.wristPIDController =
        new PIDController(PIDConstants.wristkP, PIDConstants.wristkI, PIDConstants.wristkD);
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristSubsystem.setGoal(setpoint);
    wristSubsystem.setSetpoint(new TrapezoidProfile.State(wristSubsystem.getPosition(), 0));

    SmartDashboard.putNumber("Arm Goal Position", wristSubsystem.getGoal().position);
    SmartDashboard.putNumber("Arm Setpoint Position", wristSubsystem.getSetpoint().position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentProfile =
        new TrapezoidProfile(
            wristSubsystem.getConstraints(),
            wristSubsystem.getGoal(),
            wristSubsystem.getSetpoint());

    double currentPosition = wristSubsystem.getPosition();

    TrapezoidProfile.State nextSetpoint = currentProfile.calculate(0.02);

    double feedForwardPower =
        feedforward.calculate(
                nextSetpoint.position + armSubsystem.getPosition() * 2 * Math.PI,
                nextSetpoint.velocity)
            / 12;

    wristSubsystem.setSetpoint(nextSetpoint);

    wristPIDController.setSetpoint(nextSetpoint.position);

    double armPower = wristPIDController.calculate(currentPosition);

    wristSubsystem.setPower(armPower + feedForwardPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setPower(0);
    wristSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristSubsystem.getSetpoint().position == wristSubsystem.getGoal().position;
  }
}
