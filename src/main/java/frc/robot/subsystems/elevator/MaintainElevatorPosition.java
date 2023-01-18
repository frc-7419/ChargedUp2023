// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MaintainElevatorPosition extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private double pos;
  private double kp;
  private double ff;

  private PIDController pidController;

  public MaintainElevatorPosition(ElevatorSubsystem elevatorSubsystem, double pos, double kp, double ff) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.pos = pos;
    this.kp = kp;
    this.ff = ff;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {

    pidController = new PIDController(kp, 0, 0);
    pidController.setSetpoint(pos);

    elevatorSubsystem.coast();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setPower(pidController.calculate(elevatorSubsystem.getElevatorPosition()) + ff);
    SmartDashboard.putNumber("elevatorOutput", elevatorSubsystem.getOutput());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.brake();
    elevatorSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}