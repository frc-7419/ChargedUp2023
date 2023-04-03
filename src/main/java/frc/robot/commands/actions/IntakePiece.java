// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToSetpointWithFeedforward;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpointWithFeedForward;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToSetpointWithFeedforward;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakePiece extends SequentialCommandGroup {
  /** Creates a new IntakePiece. */

  /**
   * This command will intake a game piece.
   *
   * @param elevatorSubsystem for controlling position of the elevator.
   * @param armSubsystem for controlling position of the arms.
   * @param gripperSubsystem for controlling gripper power and direction.
   * @param wristSubsystem for controlling the orientation of the gripper
   */
  public IntakePiece(

      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem,
      WristSubsystem wristSubsystem,
      NodeState intakeLocation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ElevatorToSetpointWithFeedForward(elevatorSubsystem, intakeLocation).raceWith(new WaitCommand(1)),
        Commands.sequence(
        new ArmToSetpointWithFeedforward(armSubsystem, intakeLocation).raceWith(new WaitCommand(2.5)),
        new WristToSetpointWithFeedforward(wristSubsystem, armSubsystem, intakeLocation).raceWith(new WaitCommand(1))));
  }
}
