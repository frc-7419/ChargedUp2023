// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.actions.SmartRetract;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.RunDrive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.SmartBalance;
import frc.robot.subsystems.wrist.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHigh extends SequentialCommandGroup {
  /** Creates a new AutoHigh. */
  public AutoHigh(DriveBaseSubsystem driveBaseSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem, GyroSubsystem gyroSubsystem, NodeState scoreLocation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, scoreLocation),
      new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem),
      new RunDrive(driveBaseSubsystem, -0.35).withTimeout(3.2),
        // new RunDrive(driveBaseSubsystem, 0.28).withTimeout(2.5),
        // new SmartBalance(driveBaseSubsystem, gyroSubsystem),
        new InstantCommand(driveBaseSubsystem::brake));
  }
}
