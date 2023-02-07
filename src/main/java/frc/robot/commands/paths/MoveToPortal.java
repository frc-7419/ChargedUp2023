// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.GenerateTrajectory;
import java.util.ArrayList;
import java.util.List;

public class MoveToPortal extends SequentialCommandGroup {

  private String teamColor = "";
  public MoveToPortal(DriveBaseSubsystem driveBaseSubsystem) {
    List<PathPoint> waypoints = new ArrayList<PathPoint>();

    waypoints.add(new PathPoint(new Translation2d(10.83, 0.99), Rotation2d.fromDegrees(0)));

    // TODO need to get waypoints for this
    
    if (Constants.RobotConstants.currentAlliance == DriverStation.Alliance.Blue) {
      waypoints.add(Constants.WaypointPositionConstants.kBlueMoveToPortalFirstWayPoint);
      waypoints.add(Constants.WaypointPositionConstants.kBlueMoveToPortalSecondWayPoint);
      teamColor = "BluePortal";
    }
    else if (Constants.RobotConstants.currentAlliance == DriverStation.Alliance.Red) {
      waypoints.add(Constants.WaypointPositionConstants.kRedMoveToPortalSecondWayPoint);
      teamColor = "RedPortal";
    }
    
    addCommands(
        new GenerateTrajectory(
            driveBaseSubsystem, PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints)),
        new GenerateTrajectory(
            driveBaseSubsystem,
            PathPlanner.loadPath("BlueMid", PathPlanner.getConstraintsFromPath("BlueMid"))));
  }
}
