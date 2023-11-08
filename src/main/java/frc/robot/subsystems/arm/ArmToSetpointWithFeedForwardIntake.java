// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.constants.PIDConstants;

public class ArmToSetpointWithFeedForwardIntake extends CommandBase {
  /** Creates a new ArmToSetpointWithFeedforward. */
  private ArmSubsystem armSubsystem;

  private double setpoint;
  private TrapezoidProfile currentProfile;
  private ArmFeedforward feedforward;
  private PIDController armPIDController;

  public ArmToSetpointWithFeedForwardIntake(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
   
    this.feedforward = ArmConstants.armFeedforward;
    // if (!gripperSubsystem.isHolding) {
    //   this.feedforward =
    //       new ArmFeedforward(ArmConstants.withoutConeks, ArmConstants.withoutConekg, ArmConstants.withoutConekv, ArmConstants.withoutConeka);
    //     } else if (gripperSubsystem.isHolding) {
    //   this.feedforward =
    //       new ArmFeedforward(ArmConstants.withConeks, ArmConstants.withConekg, ArmConstants.withConekv, ArmConstants.withConeka);
    // }
    this.armPIDController =
        new PIDController(0.05, 0, 0);
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.setpoint = NodeState.valueOf(SmartDashboard.getString("Single Sub State", "GROUND_INTAKE")).armSetpoint;
    armSubsystem.setGoal(setpoint);
    double armPosition = armSubsystem.getAngle();
    armSubsystem.setSetpoint(new TrapezoidProfile.State(armPosition, 0));

    SmartDashboard.putNumber("Arm Goal Position", armSubsystem.getGoal().position);
    SmartDashboard.putNumber("Arm Setpoint Position", armSubsystem.getSetpoint().position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentProfile =
        new TrapezoidProfile(
            armSubsystem.getConstraints(), armSubsystem.getGoal(), armSubsystem.getSetpoint());

    double currentPosition = armSubsystem.getAngle();

    TrapezoidProfile.State nextSetpoint = currentProfile.calculate(0.02);

    // double feedForwardPower =
    //     feedforward.calculate(nextSetpoint.position, nextSetpoint.velocity) / 12;

    armSubsystem.setSetpoint(nextSetpoint);
    
    armPIDController.setSetpoint(nextSetpoint.position);
    
    double armPower = armPIDController.calculate(currentPosition);
    double feedForwardPower = 
    Math.copySign(ArmConstants.kg * Math.cos(Units.degreesToRadians(currentPosition)), armPower)/12;

    armSubsystem.setPower(armPower);
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
    double error = armSubsystem.getGoal().position - armSubsystem.getAngle();
    boolean isAtSetpoint = Math.abs(error) <= 1;
    return isAtSetpoint;
  }
}
