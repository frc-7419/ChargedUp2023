// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SmartWrist extends CommandBase {
  private WristSubsystem wristSubsystem;
  private PIDController wristController;
  private double setpoint;

  public SmartWrist(WristSubsystem wristSubsystem, double setpoint) {
    this.wristSubsystem = wristSubsystem;
    this.setpoint = setpoint;
    wristController =
        new PIDController(PIDConstants.wristkP, PIDConstants.wristkI, PIDConstants.wristkD);
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristController.setSetpoint(setpoint);
    wristController.setTolerance(PIDConstants.wristTolerance);
  }

  @Override
  public void execute() {
    double wristPosition = wristSubsystem.getPosition();
    double output = wristController.calculate(wristPosition);
    wristSubsystem.setPower(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
    wristSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristController.atSetpoint();
  }
}
