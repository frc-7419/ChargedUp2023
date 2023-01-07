// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunLoader extends CommandBase {
  private LoaderSubsystem loaderSubsystem;
  private double power;

  public RunLoader(LoaderSubsystem loaderSubsystem, double power) {
    this.loaderSubsystem = loaderSubsystem;
    this.power = power;
    addRequirements(loaderSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    loaderSubsystem.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    loaderSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
