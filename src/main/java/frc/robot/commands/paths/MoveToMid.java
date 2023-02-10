// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.GenerateTrajectory;
import java.util.ArrayList;
import java.util.List;

public class MoveToMid extends SequentialCommandGroup {

  private String teamColor = "";
  public MoveToMid(DriveBaseSubsystem driveBaseSubsystem) {
    List<PathPoint> waypoints = new ArrayList<PathPoint>();

    // TODO need to get waypoints for this
    
    if (Constants.RobotConstants.currentAlliance == DriverStation.Alliance.Blue) {
      waypoints.add(Constants.WaypointPositionConstants.kBlueMidFirstWayPoint);
      waypoints.add(Constants.WaypointPositionConstants.kBlueMidSecondWayPoint);
      waypoints.add(Constants.WaypointPositionConstants.kBlueMidThirdWayPoint);
      teamColor = "BlueMid";
    }
    else if (Constants.RobotConstants.currentAlliance == DriverStation.Alliance.Red) {
      waypoints.add(Constants.WaypointPositionConstants.kRedMidFirstWayPoint);
      waypoints.add(Constants.WaypointPositionConstants.kRedMidSecondWayPoint);
      waypoints.add(Constants.WaypointPositionConstants.kRedMidThirdWayPoint);
      teamColor = "RedMid";
    }
    
    addCommands(
        new GenerateTrajectory(
            driveBaseSubsystem, PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints)),
        new GenerateTrajectory(
            driveBaseSubsystem,
            PathPlanner.loadPath(teamColor, PathPlanner.getConstraintsFromPath(teamColor))));
  }
}
