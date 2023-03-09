// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.constants.PIDConstants;

public class ArmToSetpointWithFeedforward extends CommandBase {
  /** Creates a new ArmToSetpointWithFeedforward. */
  private ArmSubsystem armSubsystem;

  private double setpoint;
  private TrapezoidProfile currentProfile;
  private ArmFeedforward feedforward;
  private PIDController armPIDController;

  public ArmToSetpointWithFeedforward(ArmSubsystem armSubsystem, NodeState armState) {
    this.armSubsystem = armSubsystem;
    this.setpoint = armState.armSetpoint;
    this.feedforward =
        new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);
    this.armPIDController =
        new PIDController(PIDConstants.armKp, PIDConstants.armKi, PIDConstants.armKd);
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setGoal(setpoint);
    armSubsystem.setSetpoint(new TrapezoidProfile.State(armSubsystem.getPosition(), 0));

    SmartDashboard.putNumber("Arm Goal Position", armSubsystem.getGoal().position);
    SmartDashboard.putNumber("Arm Setpoint Position", armSubsystem.getSetpoint().position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentProfile =
        new TrapezoidProfile(
            armSubsystem.getConstraints(), armSubsystem.getGoal(), armSubsystem.getSetpoint());

    double currentPosition = armSubsystem.getPosition();

    TrapezoidProfile.State nextSetpoint = currentProfile.calculate(0.02);

    double feedForwardPower =
        feedforward.calculate(nextSetpoint.position, nextSetpoint.velocity) / 12;

    armSubsystem.setSetpoint(nextSetpoint);

    armPIDController.setSetpoint(nextSetpoint.position);

    double armPower = armPIDController.calculate(currentPosition);

    armSubsystem.setPower(armPower + feedForwardPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setPower(0);
    armSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.getSetpoint().position == armSubsystem.getGoal().position;
  }
}
