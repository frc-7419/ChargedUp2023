// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class RunFeederWithJoystick extends CommandBase {
  private FeederSubsystem feederSubsystem;
  private XboxController joystick;

  public RunFeederWithJoystick(FeederSubsystem feederSubsystem, XboxController joystick) {
    this.feederSubsystem = feederSubsystem;
    this.joystick = joystick;
    addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getLeftBumper()) {
      feederSubsystem.setVoltage(-Constants.PowerConstants.FeederVoltage);
    } else if (joystick.getRightBumper()) {
      feederSubsystem.setVoltage(Constants.PowerConstants.FeederVoltage);
    } else {
      feederSubsystem.setVoltage(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
