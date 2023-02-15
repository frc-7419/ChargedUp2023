// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WaypointPositionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.GenerateTrajectory;
import java.util.ArrayList;
import java.util.List;

public class MoveToMid extends SequentialCommandGroup {

  private String pathAccordingToTeamColor = "";

  public MoveToMid(DriveBaseSubsystem driveBaseSubsystem) {
    List<PathPoint> waypoints = new ArrayList<PathPoint>();

    if (Robot.getAllianceColor().equals("Blue")) {
      waypoints.add(WaypointPositionConstants.kBlueMidFirstWayPoint);
      waypoints.add(WaypointPositionConstants.kBlueMidSecondWayPoint);
      waypoints.add(WaypointPositionConstants.kBlueMidThirdWayPoint);
      pathAccordingToTeamColor = "BlueMid";
    } else {
      waypoints.add(WaypointPositionConstants.kRedMidFirstWayPoint);
      waypoints.add(WaypointPositionConstants.kRedMidSecondWayPoint);
      waypoints.add(WaypointPositionConstants.kRedMidThirdWayPoint);
      pathAccordingToTeamColor = "RedMid";
    }

    addCommands(
        new GenerateTrajectory(
            driveBaseSubsystem, PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints)),
        new GenerateTrajectory(
            driveBaseSubsystem,
            PathPlanner.loadPath(
                pathAccordingToTeamColor,
                PathPlanner.getConstraintsFromPath(pathAccordingToTeamColor))));
  }
}
