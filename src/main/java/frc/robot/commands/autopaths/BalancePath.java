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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalancePath extends SequentialCommandGroup {
  public BalancePath(DriveBaseSubsystem driveBaseSubsystem) {
    String allianceSide = RobotConstants.currentAllianceSide;
    String pathName = "Balance Mid";
    Alliance currentAlliance = RobotConstants.currentAlliance;
    PathPlannerTrajectory balancePath =
        PathPlanner.loadPath(pathName, PathPlanner.getConstraintsFromPath(pathName));
    PathPlannerTrajectory.transformTrajectoryForAlliance(balancePath, currentAlliance);

    addCommands(new GenerateTrajectory(driveBaseSubsystem, balancePath));
  }
}
