// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.WaypointPositionConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.GenerateTrajectory;
import java.util.ArrayList;
import java.util.List;

public class TurnToAngle extends SequentialCommandGroup {
  private double angleToRotate;
  private double currentXPose;
  private double currentYPose;
  private double currentRotationDegrees;
  List<PathPoint> waypoints;
  public TurnToAngle(DriveBaseSubsystem driveBaseSubsystem, double angle) {
    currentXPose = driveBaseSubsystem.getCtrlsPoseEstimate().getX();
    currentYPose = driveBaseSubsystem.getCtrlsPoseEstimate().getY();
    currentRotationDegrees = driveBaseSubsystem.getCtrlsPoseEstimate().getRotation().getDegrees();
    waypoints = new ArrayList<PathPoint>();
    angleToRotate = (angle + currentRotationDegrees) % 360;
    
    waypoints.add(new PathPoint(
      new Translation2d(currentXPose, currentYPose), Rotation2d.fromDegrees(currentRotationDegrees)));
      waypoints.add(new PathPoint(
      new Translation2d(currentXPose, currentYPose),
      Rotation2d.fromDegrees(angleToRotate)));

    addCommands(
        new GenerateTrajectory(
            driveBaseSubsystem, PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints)));
  }
}
