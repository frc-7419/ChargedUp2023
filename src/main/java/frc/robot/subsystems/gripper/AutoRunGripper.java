// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.GripperConstants;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.subsystems.led.LedSubsystem;

public class AutoRunGripper extends CommandBase {
  /** Creates a new SmartRunGripper. */
  GripperSubsystem gripperSubsystem;

  GripperState mode;
  LedSubsystem ledSubsystem;
  private double lastTimeStamp;
  private boolean holdMode = false;
  private boolean isIntaking = false;
  private boolean isOuttaking = false;

  public AutoRunGripper(
      GripperSubsystem gripperSubsystem, GripperState mode, LedSubsystem ledSubsystem) {
    this.mode = mode;
    this.gripperSubsystem = gripperSubsystem;
    this.ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripperSubsystem.coast();
    lastTimeStamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripperSubsystem.isHolding = holdMode;

    if (!isOuttaking && mode == GripperState.INTAKE && !holdMode) {
      isIntaking = !isIntaking;
      if (isIntaking) {
        holdMode = false;
      }
    }
    if (!isIntaking && mode == GripperState.SCORE) {
      isOuttaking = !isOuttaking;
      if (isOuttaking) {
        holdMode = false;
      }
    }

    if (holdMode) {
      gripperSubsystem.setIntakePower(GripperConstants.gripperFeedforward);
      gripperSubsystem.brake();
      ledSubsystem.setLEDGreen();
    } else if (isIntaking) {
      gripperSubsystem.coast();
      gripperSubsystem.setIntakePower(GripperConstants.gripperPower);
      ledSubsystem.setLEDRed();
      double currentTimeStamp = Timer.getFPGATimestamp();
      double timePassed = currentTimeStamp - lastTimeStamp;
      boolean isStalling = gripperSubsystem.getVelocity() < GripperConstants.stallVelocityThreshold;
      boolean didDelay = timePassed > GripperConstants.gripperDelaySeconds;
      if (isStalling && didDelay) {
        // Hold mode won't be set to true unless we run it for 0.5 seconds to get the motor up to
        // speed
        holdMode = true;
        isIntaking = false;
      }
    } else if (isOuttaking) {
      gripperSubsystem.coast();
      gripperSubsystem.setOuttakePower(GripperConstants.gripperPower);
      ledSubsystem.setLEDBlue();
    } else {
      gripperSubsystem.setPower(0);
      lastTimeStamp = Timer.getFPGATimestamp();
      gripperSubsystem.brake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
