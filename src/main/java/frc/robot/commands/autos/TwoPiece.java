// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.AutoIntakePiece;
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.autopaths.TwoPiecePath;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import java.util.HashMap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPiece extends SequentialCommandGroup {
  /** Creates a new OnePieceMobility. */
  public TwoPiece(
      DriveBaseSubsystem driveBaseSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem,
      GripperSubsystem gripperSubsystem) {
    HashMap<String, Command> eventMap = new HashMap<String, Command>();
    Alliance alliance = RobotConstants.currentAlliance;
    String allianceSide = RobotConstants.currentAllianceSide;
    String pathName = "Two Piece" + allianceSide;
    eventMap.put(
        "Intake Piece",
        new AutoIntakePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.RESET));

    eventMap.put(
        "Score Piece High",
        new AutoScorePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.HIGH));

    PathPlannerTrajectory twoPiece =
        PathPlanner.loadPath(pathName, PathPlanner.getConstraintsFromPath(pathName));
        PathPlannerTrajectory.transformTrajectoryForAlliance(twoPiece,alliance);
    addCommands(
        new FollowPathWithEvents(
            new TwoPiecePath(driveBaseSubsystem), twoPiece.getMarkers(), eventMap));
  }
}
