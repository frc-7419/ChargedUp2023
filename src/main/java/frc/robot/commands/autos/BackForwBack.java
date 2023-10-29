// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.RunDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackForwBack extends SequentialCommandGroup {
  /** Creates a new Mobility. */
  public BackForwBack(DriveBaseSubsystem driveBaseSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new RunDrive(driveBaseSubsystem, 0.4).withTimeout(0.5),
        // new RunDrive(driveBaseSubsystem, -0.2).withTimeout(0.75),
        new RunDrive(driveBaseSubsystem, -0.25).withTimeout(3.5));
         
  }
}