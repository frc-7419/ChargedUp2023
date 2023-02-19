// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;

public class SetElevatorPosition extends CommandBase {
  /** Creates a new SetElevatorPosition. */
  private PIDController pidController;

  private ElevatorSubsystem elevatorSubsystem;
  private double setpoint;

  public SetElevatorPosition(ElevatorSubsystem elevatorSubsystem, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    pidController =
        new PIDController(
            PIDConstants.ElevatorKp, PIDConstants.ElevatorKi, PIDConstants.ElevatorKd);
    this.elevatorSubsystem = elevatorSubsystem;
    this.setpoint = setpoint;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setTolerance(PIDConstants.ElevatorKTolerance);
    pidController.setSetpoint(setpoint);
    elevatorSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = elevatorSubsystem.getElevatorPosition();
    double pidOutput = pidController.calculate(currentPosition);
    elevatorSubsystem.setPower(pidOutput);

    double error = pidController.getPositionError();
    SmartDashboard.putNumber("setElevatorPosition Error", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setPower(0);
    elevatorSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean setpointReached = pidController.atSetpoint();
    return setpointReached;
  }
}
