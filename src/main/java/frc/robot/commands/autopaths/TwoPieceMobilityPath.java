// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autopaths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.GenerateTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceMobilityPath extends SequentialCommandGroup {
  /** Creates a new OnePieceMobilityPath. */
  public TwoPieceMobilityPath(DriveBaseSubsystem driveBaseSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory twoPieceMoblity =
        PathPlanner.loadPath(
            "Two Piece + Mobility", PathPlanner.getConstraintsFromPath("Two Piece + Mobility"));
    addCommands(new GenerateTrajectory(driveBaseSubsystem, twoPieceMoblity));
  }
}
