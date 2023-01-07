// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class RunIntakeAndLoaderWithJoystick extends CommandBase {
  private XboxController joystick;
  private IntakeSubsystem intakeSubsystem;
  private LoaderSubsystem loaderSubsystem;

  public RunIntakeAndLoaderWithJoystick(XboxController joystick, IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem) {
    this.joystick = joystick;
    this.intakeSubsystem = intakeSubsystem;
    this.loaderSubsystem = loaderSubsystem;
    addRequirements(intakeSubsystem, loaderSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (joystick.getRightTriggerAxis() > 0) {
      intakeSubsystem.setPower(1);
      loaderSubsystem.setPower(0.6);
    }
    else if (joystick.getLeftTriggerAxis() > 0) {
        intakeSubsystem.setPower(-1);
        loaderSubsystem.setPower(-0.4);
      }
    else {
      intakeSubsystem.setPower(0);
      loaderSubsystem.setPower(0);
      // loaderSubsystem.brake();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
