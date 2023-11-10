// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToSetpointWithFeedforward;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpointWithFeedForward;
// import frc.robot.subsystems.elevator.ElevatorToSetpoint;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripper;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToSetpointWithFeedforward;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartRetract extends SequentialCommandGroup {
  /** Creates a new ScorePiece. */

  /**
   * This command will score a game piece.
   *
   * @param elevatorSubsystem for controlling position of the elevator.
   * @param armSubsystem for controlling position of the arms.
   * @param gripperSubsystem for controlling orientation of the gripper.
   */
  public SmartRetract(
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.parallel(
      new ArmToSetpointWithFeedforward(armSubsystem, NodeState.RESET).raceWith(new WaitCommand(1.5)),
          new WristToSetpointWithFeedforward(wristSubsystem, armSubsystem, NodeState.RESET).beforeStarting(new WaitCommand(1)).raceWith(new WaitCommand(1)),
        new ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.RESET).raceWith(new WaitCommand(2))).withTimeout(5));
  }
  public SmartRetract(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.parallel(
          new WristToSetpointWithFeedforward(wristSubsystem, armSubsystem, NodeState.RESET).raceWith(new WaitCommand(1))));
  }
}



