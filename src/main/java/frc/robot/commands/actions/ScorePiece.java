// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants.State;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.SmartArm;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScorePiece extends SequentialCommandGroup {
  /** Creates a new ScorePiece. */
  public ScorePiece(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       // once we implement set points for elevator, this will reset the position of the elevator to its lowest point
      // new SetElevatorPosition(elevatorSubsystem, ElevatorConstants.scoreSetpoint),

      new SmartArm(armSubsystem, ArmConstants.scoreSetpoint),

      // setting position of wrist for intaking orientation
      // new SetWristPosition(wristSubsystem, WristConstants.scoreSetpoint)

      // running gripper
      new RunGripper(gripperSubsystem, State.SCORE, 1),

      // bringing arms back
      new SmartArm(armSubsystem, ArmConstants.resetSetpoint)

      // resetting position of wrist for intaking orientation
      // new SetWristPosition(wristSubsystem, WristConstants.resetSetpoint)

    );
  }
}
