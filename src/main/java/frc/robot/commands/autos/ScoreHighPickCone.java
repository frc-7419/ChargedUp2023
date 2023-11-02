// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToSetpointWithFeedforward;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpointWithFeedForward;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripper;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToSetpointWithFeedforward;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.RunDrive;
import frc.robot.subsystems.drive.GenerateTrajectory;
import frc.robot.commands.actions.AutoIntakePiece;
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.actions.SmartRetract;
import frc.robot.commands.autopaths.OnePiecePath;
import frc.robot.constants.NodeConstants.NodeState;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.RobotConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighPickCone extends SequentialCommandGroup {
  /** Creates a new IntakePiece. */

  /**
   * This command will intake a game piece.
   *
   * @param elevatorSubsystem for controlling position of the elevator.
   * @param armSubsystem      for controlling position of the arms.
   * @param gripperSubsystem  for controlling gripper power and direction.
   * @param wristSubsystem    for controlling the orientation of the gripper
   */
  public ScoreHighPickCone(

      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem,
      WristSubsystem wristSubsystem,
      GripperSubsystem gripperSubsystem,
      NodeState scoreLocation,
      DriveBaseSubsystem driveBaseSubsystem,
      NodeState intakeLocation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    HashMap<String, Command> eventMap = new HashMap<String, Command>();

    String allianceSide = RobotConstants.currentAllianceSide;
    String pathName = "One Piece" + allianceSide;
    Alliance alliance = RobotConstants.currentAlliance;

    eventMap.put(
        "Score Piece High",
        new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, NodeState.HIGH));
    eventMap.put(
        "Retract Intake", new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem));
    eventMap.put(
        "Intake Piece",
        new AutoIntakePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.GROUND));

    PathPlannerTrajectory onePiece =
        PathPlanner.loadPath(pathName, PathPlanner.getConstraintsFromPath(pathName));
    PathPlannerTrajectory.transformTrajectoryForAlliance(onePiece, alliance);

    addCommands(
        Commands.parallel(
            new RunGripper(gripperSubsystem, GripperState.HOLD).raceWith(new WaitCommand(4)),
            new ElevatorToSetpointWithFeedForward(elevatorSubsystem, scoreLocation).raceWith(new WaitCommand(3)),
            new ArmToSetpointWithFeedforward(armSubsystem, scoreLocation).raceWith(new WaitCommand(3))),
        new WristToSetpointWithFeedforward(wristSubsystem, armSubsystem, scoreLocation).raceWith(new WaitCommand(1.5)),
        new RunGripper(gripperSubsystem, GripperState.SCORE).withTimeout(1),
        new InstantCommand(gripperSubsystem::stop),
        new FollowPathWithEvents(
            new OnePiecePath(driveBaseSubsystem), onePiece.getMarkers(), eventMap));
  }
}
