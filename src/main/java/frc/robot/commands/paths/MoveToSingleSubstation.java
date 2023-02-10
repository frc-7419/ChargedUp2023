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

/**
 * Use path planner to move the robot from its current position to the single substation. Do note that the constants used
 * in the waypoints are created in such a way that the robot must be past the charge station in order to avoid hitting it accidentally/**

 */
public class MoveToSingleSubstation extends SequentialCommandGroup {
  private String teamColorWithPath = "";

  public MoveToSingleSubstation(DriveBaseSubsystem driveBaseSubsystem) {

    // TODO add waypoints for single substation
    List<PathPoint> waypoints = new ArrayList<PathPoint>();
    
    // TODO uncomment when we test other alliances
    
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      waypoints.add(Constants.WaypointPositionConstants.kBlueSubstationFirstWayPoint);
      waypoints.add(Constants.WaypointPositionConstants.kBlueSubstationSecondWayPoint);
      teamColorWithPath = "BlueSingleSubstation";
    }
    else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      waypoints.add(Constants.WaypointPositionConstants.kRedSubstationFirstWayPoint);
      waypoints.add(Constants.WaypointPositionConstants.kRedSubstationSecondWayPoint);
      teamColorWithPath = "RedSingleSubstation";
    }

    addCommands(
        new GenerateTrajectory(
            driveBaseSubsystem, PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints)),
        new GenerateTrajectory(
            driveBaseSubsystem,
            PathPlanner.loadPath(
                teamColorWithPath, PathPlanner.getConstraintsFromPath(teamColorWithPath))));
  }
}
