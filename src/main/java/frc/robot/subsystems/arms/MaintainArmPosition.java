// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MaintainArmPosition extends CommandBase {
  /** Creates a new SetPosition. */
  private ArmsSubsystem armSubsystem;
  private double pos;
  private double kP;
  private PIDController pidController;
  private double ff;


  public MaintainArmPosition(ArmsSubsystem armSubsystem, double pos, double kP, double ff) {
    this.armSubsystem = armSubsystem;
    this.pos = pos;
    this.kP = kP;
    this.ff = ff;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // kP = SmartDashboard.getNumber("armKp", 0.0001);
    // pos = SmartDashboard.getNumber("armSetpoint", 2);
    // ff = SmartDashboard.getNumber("armFf", 0);

    pidController = new PIDController(kP, 0, 0);
    pidController.setSetpoint(pos);
    pidController.setTolerance(0.2);

    armSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(armSubsystem.getPosition()) + ff;
    armSubsystem.setPower(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}