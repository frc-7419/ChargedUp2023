// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveWristWithJoystick extends CommandBase {
  private WristSubsystem wristSubsystem;
  private XboxController joystick;

  public MoveWristWithJoystick(WristSubsystem wristSubsystem, XboxController joystick) {
    this.wristSubsystem = wristSubsystem;
    this.joystick = joystick;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristSubsystem.coast();
  }

  @Override
  public void execute() {
    double power = joystick.getLeftY();
    wristSubsystem.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
    wristSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
