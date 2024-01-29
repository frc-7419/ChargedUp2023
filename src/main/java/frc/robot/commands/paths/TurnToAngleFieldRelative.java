// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.paths;

// import com.pathplanner.lib.PathPoint;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.drive.DriveBaseSubsystem;
// import frc.robot.subsystems.drive.GenerateTrajectoryFromCurrentPose;
// import java.util.ArrayList;
// import java.util.List;

// public class TurnToAngleFieldRelative extends SequentialCommandGroup {
//   private double currentXPose;
//   private double currentYPose;
//   private double currentRotationDegrees;
//   List<PathPoint> waypoints;

//   public TurnToAngleFieldRelative(DriveBaseSubsystem driveBaseSubsystem, double angle) {
//     currentXPose = driveBaseSubsystem.getCtrlsPoseEstimate().getX();
//     currentYPose = driveBaseSubsystem.getCtrlsPoseEstimate().getY();
//     currentRotationDegrees = driveBaseSubsystem.getCtrlsPoseEstimate().getRotation().getDegrees();
//     SmartDashboard.putNumber("turn odo x", currentXPose);
//     SmartDashboard.putNumber("turn odo y", currentYPose);
//     SmartDashboard.putNumber("turn odo theta", currentRotationDegrees);
//     waypoints = new ArrayList<PathPoint>();
//     waypoints.add(
//         new PathPoint(
//             new Translation2d(currentXPose, currentYPose),
//             Rotation2d.fromDegrees(currentRotationDegrees)));
//     waypoints.add(
//         new PathPoint(
//             new Translation2d(currentXPose, currentYPose), Rotation2d.fromDegrees(angle)));

//     addCommands(new GenerateTrajectoryFromCurrentPose(driveBaseSubsystem, waypoints));
//   }
// }
