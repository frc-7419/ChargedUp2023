// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.IntakePiece;
import frc.robot.commands.actions.ScorePiece;
import frc.robot.commands.autopaths.OnePieceMobilityPath;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import java.util.HashMap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceMobility extends SequentialCommandGroup {
  /** Creates a new OnePieceMobility. */
  public OnePieceMobility(
      DriveBaseSubsystem driveBaseSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem,
      GripperSubsystem gripperSubsystem) {
    HashMap<String, Command> eventMap = new HashMap<String, Command>();

    eventMap.put(
        "Intake Piece", new IntakePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.RESET));

    eventMap.put(
        "Score Piece",
        new ScorePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.HIGH));

    PathPlannerTrajectory onePieceMobility =
        PathPlanner.loadPath(
            "One Piece + Mobility", PathPlanner.getConstraintsFromPath("One Piece + Mobility"));

    addCommands(
        new FollowPathWithEvents(
            new OnePieceMobilityPath(driveBaseSubsystem), onePieceMobility.getMarkers(), eventMap));
  }
}
