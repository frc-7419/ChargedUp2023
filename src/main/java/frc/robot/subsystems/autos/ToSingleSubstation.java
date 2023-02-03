// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SamplePath;

public class ToSingleSubstation extends SequentialCommandGroup {
  public ToSingleSubstation(DriveBaseSubsystem driveBaseSubsystem) {
    List<PathPoint> waypoints = new ArrayList<PathPoint>();
    String teamColor = "";
    waypoints.add(new PathPoint(new Translation2d(driveBaseSubsystem.getCtrlsPoseEstimate().getX(), driveBaseSubsystem.getCtrlsPoseEstimate().getY()), driveBaseSubsystem.getCtrlsPoseEstimate().getRotation()));
    // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
    //   waypoints.add(Constants.WaypointConstants.kBluePoint);
    //   teamColor = "BlueSingleSubstation";
    // }
    // else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
    //   waypoints.add(Constants.WaypointConstants.kRedPoint); 
    //   teamColor = "RedSingleSubstation";
    // }
    addCommands(
        new SamplePath(driveBaseSubsystem, PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints)),
        new SamplePath(driveBaseSubsystem, PathPlanner.loadPath(teamColor, PathPlanner.getConstraintsFromPath(teamColor))
      ));
  }
}
