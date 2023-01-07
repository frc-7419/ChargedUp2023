// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArmsWithJoystick extends CommandBase {
  private ArmsSubsystem armsSubsystem;
  private XboxController joystick;
  /** Creates a new RunArmsWithJoystick. */
  public RunArmsWithJoystick(ArmsSubsystem armsSubsystem, XboxController joystick) {
    this.armsSubsystem = armsSubsystem;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.armsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getRightY() != 0) {
      armsSubsystem.coast();
      armsSubsystem.setPower(joystick.getRightY() * 0.12);
    } else {
      armsSubsystem.setPower(0);
      armsSubsystem.brake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armsSubsystem.setPower(0);
    armsSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

