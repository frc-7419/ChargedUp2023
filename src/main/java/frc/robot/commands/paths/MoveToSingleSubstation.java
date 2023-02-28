// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.WaypointPositionConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.GenerateTrajectory;
import java.util.ArrayList;
import java.util.List;

public class MoveToSingleSubstation extends SequentialCommandGroup {
  private String teamColorAccordingToPath = "";

  public MoveToSingleSubstation(DriveBaseSubsystem driveBaseSubsystem) {
    String allianceColor = Robot.getAllianceColor();
    List<PathPoint> waypoints = new ArrayList<PathPoint>();

    if (allianceColor.equals("Blue")) {
      waypoints.add(WaypointPositionConstants.kBlueSubstationFirstWayPoint);
      waypoints.add(WaypointPositionConstants.kBlueSubstationSecondWayPoint);
      teamColorAccordingToPath = "BlueSingleSubstation";
    } else {
      waypoints.add(WaypointPositionConstants.kRedSubstationFirstWayPoint);
      waypoints.add(WaypointPositionConstants.kRedSubstationSecondWayPoint);
      teamColorAccordingToPath = "RedSingleSubstation";
    }

    addCommands(
        new GenerateTrajectory(
            driveBaseSubsystem, PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints)),
        new GenerateTrajectory(
            driveBaseSubsystem,
            PathPlanner.loadPath(
                teamColorAccordingToPath,
                PathPlanner.getConstraintsFromPath(teamColorAccordingToPath))));
  }
}
