// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.RunDrive;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OuttakeMove extends SequentialCommandGroup {
  /** Creates a new Mobility. */
  public OuttakeMove(DriveBaseSubsystem driveBaseSubsystem, GripperSubsystem gripperSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RunGripper(gripperSubsystem, GripperState.SCORE).withTimeout(1),
        new RunDrive(driveBaseSubsystem, 0.8).withTimeout(3.4));
  }
}
