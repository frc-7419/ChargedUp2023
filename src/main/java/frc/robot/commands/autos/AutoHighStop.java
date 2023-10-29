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
import frc.robot.commands.actions.SmartRetractAuto;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToSetpointWithFeedforward;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.RunDrive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpointWithFeedForward;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripper;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.SmartBalance;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToSetpointWithFeedforward;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHighStop extends SequentialCommandGroup {
  /** Creates a new AutoHigh. */
  public AutoHighStop(DriveBaseSubsystem driveBaseSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem, GyroSubsystem gyroSubsystem, NodeState scoreLocation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
      {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(gyroSubsystem::zeroYaw),
        Commands.parallel(
          new RunGripper(gripperSubsystem, GripperState.HOLD).raceWith(new WaitCommand(4)),
        new ElevatorToSetpointWithFeedForward(elevatorSubsystem, scoreLocation).raceWith(new WaitCommand(2)),
        new ArmToSetpointWithFeedforward(armSubsystem, scoreLocation).raceWith(new WaitCommand(2.5)),
        new WristToSetpointWithFeedforward(wristSubsystem, armSubsystem, scoreLocation).raceWith(new WaitCommand(1))
        ),
        new RunGripper(gripperSubsystem, GripperState.SCORE).withTimeout(1),
        new InstantCommand(gripperSubsystem::stop),
    new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem)
    // new RunDrive(driveBaseSubsystem, 0.3).withTimeout(0.2)
    );
    
  }
      
        // // new RunDrive(driveBaseSubsystem, 0.28).withTimeout(2.8),
        // new SmartBalance(driveBaseSubsystem, gyroSubsystem),
        // new InstantCommand(driveBaseSubsystem::brake));
  }
}
