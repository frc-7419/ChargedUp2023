// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autos;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.actions.AutoScorePiece;
// import frc.robot.commands.actions.AutoScorePiece;
// import frc.robot.commands.actions.SmartRetract;
// import frc.robot.commands.autopaths.OnePiecePath;
// import frc.robot.constants.NodeConstants.NodeState;
// import frc.robot.constants.RobotConstants;
// import frc.robot.constants.GripperConstants.GripperState;
// import frc.robot.subsystems.arm.ArmSubsystem;
// import frc.robot.subsystems.drive.DriveBaseSubsystem;
// import frc.robot.subsystems.elevator.ElevatorSubsystem;
// import frc.robot.subsystems.gripper.GripperSubsystem;
// import frc.robot.subsystems.state.StateMachine;
// import frc.robot.subsystems.wrist.WristSubsystem;

// import java.util.HashMap;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class OnePiece extends SequentialCommandGroup {
//   /** Creates a new OnePieceMobility. */
//   public OnePiece(
//       DriveBaseSubsystem driveBaseSubsystem,
//       ElevatorSubsystem elevatorSubsystem,
//       ArmSubsystem armSubsystem,
//       GripperSubsystem gripperSubsystem,
//       WristSubsystem wristSubsystem, StateMachine stateMachine) {
//     HashMap<String, Command> eventMap = new HashMap<String, Command>();

//     Alliance alliance = RobotConstants.currentAlliance;
//     String allianceSide = RobotConstants.currentAllianceSide;
//     String pathName = "One Piece " + allianceSide;

//     eventMap.put(
//         "Score Piece High",
//         new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, NodeState.HIGH, GripperState.SCORE_CONE, stateMachine));
//     eventMap.put(
//         "Retract Intake", new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem));
//     eventMap.put(
//         "Intake Piece",
//         new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, NodeState.GROUND, GripperState.SCORE_CONE, stateMachine));

//     PathPlannerTrajectory onePiece =
//         PathPlanner.loadPath(pathName, PathPlanner.getConstraintsFromPath(pathName));
//     PathPlannerTrajectory.transformTrajectoryForAlliance(onePiece, alliance);

//     addCommands(
//         new FollowPathWithEvents(
//             new OnePiecePath(driveBaseSubsystem), onePiece.getMarkers(), eventMap));
//   }
// }
