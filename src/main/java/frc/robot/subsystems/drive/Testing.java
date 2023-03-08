// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Testing extends CommandBase {
  /** Creates a new Testing. */
  private DriveBaseSubsystem driveBaseSubsystem;

  private XboxController joystick;

  public Testing(DriveBaseSubsystem driveBaseSubsystem, XboxController joystick) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.joystick = joystick;
    addRequirements(driveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getRightBumper()) {
      driveBaseSubsystem.setTopPower(0.1);
    } else if (joystick.getLeftBumper()) {
      driveBaseSubsystem.setBottomPower(0.1);
    } else {
      driveBaseSubsystem.setAllPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setAllPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
