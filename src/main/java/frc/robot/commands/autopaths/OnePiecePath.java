// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autopaths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.GenerateTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePiecePath extends SequentialCommandGroup {
  public OnePiecePath(DriveBaseSubsystem driveBaseSubsystem) {
    String allianceSide = RobotConstants.currentAllianceSide;
    String path = allianceSide + " One Piece";
    Alliance currentAlliance = RobotConstants.currentAlliance;
    PathPlannerTrajectory onePiecePath =
        PathPlanner.loadPath(
            "One Piece", PathPlanner.getConstraintsFromPath("One Piece"));
    PathPlannerTrajectory.transformTrajectoryForAlliance(onePiecePath, currentAlliance);


    Translation2d translation2d = new Translation2d(0,3.96);
    Rotation2d rotation2d = new Rotation2d(Units.degreesToRadians(180));
    Transform2d transform2d = new Transform2d(translation2d, rotation2d);

    onePiecePath.transformBy(transform2d);
    addCommands(new GenerateTrajectory(driveBaseSubsystem, onePiecePath));
  }
}
 