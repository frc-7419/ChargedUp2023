// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autopaths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.GenerateTrajectory;

public class ThreePiecePath extends SequentialCommandGroup {
  public ThreePiecePath(DriveBaseSubsystem driveBaseSubsystem) {
    String allianceSide = RobotConstants.currentAllianceSide;
    String pathName = "Three Piece" + allianceSide;
    Alliance currentAlliance = RobotConstants.currentAlliance;
    PathPlannerTrajectory threePiecePath =
        PathPlanner.loadPath(pathName, PathPlanner.getConstraintsFromPath(pathName));
    PathPlannerTrajectory.transformTrajectoryForAlliance(threePiecePath, currentAlliance);
    addCommands(new GenerateTrajectory(driveBaseSubsystem, threePiecePath));
  }
}
