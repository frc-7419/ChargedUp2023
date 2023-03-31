// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.RunDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MobilityBalance extends SequentialCommandGroup {
  /** Creates a new Mobility. */
  public MobilityBalance(DriveBaseSubsystem driveBaseSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RunDrive(driveBaseSubsystem, 0.8).withTimeout(0.2),
        new RunDrive(driveBaseSubsystem, -0.4).withTimeout(1.5),
        new RunDrive(driveBaseSubsystem, 0.35).withTimeout(3.2),
        new RunDrive(driveBaseSubsystem, -0.3).withTimeout(2),
        new InstantCommand(driveBaseSubsystem::brake));
  }
}
