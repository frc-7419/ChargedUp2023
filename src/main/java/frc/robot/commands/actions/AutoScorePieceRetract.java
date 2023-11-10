// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToSetpointWithFeedforward;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpointWithFeedForward;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.gripper.RunGripper;
import frc.robot.subsystems.state.StateMachine;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScorePieceRetract extends SequentialCommandGroup {
  /** Creates a new IntakePiece. */

  /**
   * This command will intake a game piece.
   *
   * @param elevatorSubsystem for controlling position of the elevator.
   * @param armSubsystem for controlling position of the arms.
   * @param gripperSubsystem for controlling orientation of the gripper.
   */
  public AutoScorePieceRetract(
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem,
      WristSubsystem wristSubsystem,
      GripperSubsystem gripperSubsystem,
      NodeState intakeLocation, GripperState gripperDirection, StateMachine stateMachine) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, intakeLocation),
      new RunGripper(gripperSubsystem, false, stateMachine).withTimeout(0.3),
      new InstantCommand(gripperSubsystem::stop),
      new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem));
  }
}
