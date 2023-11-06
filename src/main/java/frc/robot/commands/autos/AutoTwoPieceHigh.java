// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.actions.SmartRetract;
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
import frc.robot.subsystems.wrist.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoPieceHigh extends SequentialCommandGroup {
  /** Creates a new AutoHigh. */
  public AutoTwoPieceHigh(DriveBaseSubsystem driveBaseSubsystem, ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem,
      GyroSubsystem gyroSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.parallel(new InstantCommand(armSubsystem::zeroEncoder), new InstantCommand(wristSubsystem::zeroEncoder)),
        new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, NodeState.HIGH, GripperState.SCORE_CONE).raceWith(new WaitCommand(3)),
        new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem).raceWith(new WaitCommand(3)),
        // new RunDrive(driveBaseSubsystem, -0.35).withTimeout(3.2),
        new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, NodeState.GROUND_INTAKE, GripperState.INTAKE_CUBE).raceWith(new WaitCommand(5)),
        new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem).raceWith(new WaitCommand(3)),
        new RunDrive(driveBaseSubsystem, -0.05,0.05).withTimeout(0.5),
        new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, NodeState.LOW, GripperState.SCORE_CUBE).raceWith(new WaitCommand(2)),
        new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem).raceWith(new WaitCommand(3)));
  }
}
