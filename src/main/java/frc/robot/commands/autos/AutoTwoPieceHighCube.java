// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.AutoIntakePiece;
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.actions.AutoScorePieceRetract;
import frc.robot.commands.actions.ScorePiece;
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.actions.SmartRetract;
import frc.robot.commands.actions.ZeroSensors;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.constants.NodeConstants.PieceState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.RunDrive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpointWithFeedForward;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripper;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.SmartBalance;
import frc.robot.subsystems.state.StateMachine;
import frc.robot.subsystems.wrist.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoPieceHighCube extends SequentialCommandGroup {
  /** Creates a new AutoHigh. */
  public AutoTwoPieceHighCube(DriveBaseSubsystem driveBaseSubsystem, ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem,
      GyroSubsystem gyroSubsystem, StateMachine stateMachine) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ZeroSensors(elevatorSubsystem, armSubsystem, wristSubsystem),
        new InstantCommand(stateMachine::setPieceStateCube),
        new InstantCommand(driveBaseSubsystem::coast),
        new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, NodeState.HIGH,
            GripperState.SCORE_CUBE, stateMachine),
        Commands.parallel(
        new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, NodeState.RESET).withTimeout(3),
        new RunDrive(driveBaseSubsystem, -0.662, -0.662).beforeStarting(new WaitCommand(1.5)).withTimeout(3)),
        new InstantCommand(stateMachine::setPieceStateCube),
        Commands.parallel(
        new RunDrive(driveBaseSubsystem, -0.14, -0.14).withTimeout(1.5),
        new AutoIntakePiece(elevatorSubsystem, stateMachine, armSubsystem, gripperSubsystem, wristSubsystem).withTimeout(2.7)),
        new RunDrive(driveBaseSubsystem, 0.65, 0.65).withTimeout(1.71),
        // new RunDrive(driveBaseSubsystem, -0.157, 0.157).withTimeout(0.4),
        Commands.parallel(
        new RunDrive(driveBaseSubsystem, 0.12, 0.12).withTimeout(0.6),
        new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, NodeState.LOW).withTimeout(3)),
        new RunGripper(gripperSubsystem, GripperState.SCORE_CUBE, stateMachine).withTimeout(0.5),
        new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, NodeState.RESET));
  }
}
