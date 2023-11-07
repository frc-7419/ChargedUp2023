// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.actions.ScorePiece;
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.actions.SmartRetract;
import frc.robot.commands.actions.ZeroSensors;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.RunDrive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripper;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.SmartBalance;
import frc.robot.subsystems.state.StateMachine;
import frc.robot.subsystems.wrist.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoPieceHigh extends SequentialCommandGroup {
  /** Creates a new AutoHigh. */
  public AutoTwoPieceHigh(DriveBaseSubsystem driveBaseSubsystem, ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem,
      GyroSubsystem gyroSubsystem, StateMachine stateMachine) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ZeroSensors(elevatorSubsystem, armSubsystem, wristSubsystem),
        new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, NodeState.HIGH,
            GripperState.SCORE_CONE, stateMachine),
        new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, NodeState.RESET),
        // new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem),
        // new RunDrive(driveBaseSubsystem, -0.35).withTimeout(3.2),
        // new RunDrive(driveBaseSubsystem, -0.1).withTimeout(3.2),
        new RunDrive(driveBaseSubsystem, -0.1).withTimeout(0.3),
        new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, NodeState.GROUND_INTAKE,
            GripperState.INTAKE_CUBE, stateMachine).withTimeout(4),
        // new RunDrive(driveBaseSubsystem, 0.1).withTimeout(3.2),
        new RunDrive(driveBaseSubsystem, 0.12, -0.12).withTimeout(1),
        new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, NodeState.LOW,
            GripperState.SCORE_CUBE, stateMachine),
        // new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem));
        new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, NodeState.RESET));
  }
}
