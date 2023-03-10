// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.AutoIntakePiece;
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.actions.IntakePiece;
import frc.robot.commands.actions.ScorePiece;
import frc.robot.commands.autopaths.TwoPieceMobilityPath;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import java.util.HashMap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceMobility extends SequentialCommandGroup {
  /** Creates a new OnePieceMobility. */
  public TwoPieceMobility(
      DriveBaseSubsystem driveBaseSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem,
      GripperSubsystem gripperSubsystem) {
    HashMap<String, Command> eventMap = new HashMap<String, Command>();

    eventMap.put(
        "Intake Piece",
        new AutoIntakePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.RESET));

    eventMap.put(
        "Score Piece",
        new AutoScorePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.HIGH));

    PathPlannerTrajectory twoPieceMoblity =
        PathPlanner.loadPath(
            "Two Piece + Mobility", PathPlanner.getConstraintsFromPath("Two Piece + Mobility"));

    addCommands(
        new FollowPathWithEvents(
            new TwoPieceMobilityPath(driveBaseSubsystem), twoPieceMoblity.getMarkers(), eventMap));
  }
}
