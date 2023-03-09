// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.GripperConstants;

public class RunGripperWithJoystick extends CommandBase {
  private GripperSubsystem gripperSubsystem;
  private XboxController joystick;
  private double lastTimeStamp;
  private Boolean holdMode = false;

  public RunGripperWithJoystick(GripperSubsystem gripperSubsystem, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripperSubsystem = gripperSubsystem;
    this.joystick = joystick;
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
    if (joystick.getRightBumper() && holdMode == false) {
      gripperSubsystem.coast();
      gripperSubsystem.setIntakePower(GripperConstants.gripperPower);
      double currentTimeStamp = Timer.getFPGATimestamp();
      double timePassed = currentTimeStamp - lastTimeStamp;
      boolean isStalling = gripperSubsystem.getVelocity() < GripperConstants.stallVelocityThreshold;
      boolean didDelay = timePassed > GripperConstants.gripperDelaySeconds;
      if (isStalling && didDelay) { 
        // Hold mode won't be set to true unless we run it for 0.5 seconds to get the motor up to speed
        holdMode = true;
      }
    } else if (joystick.getLeftBumper()) {
      gripperSubsystem.coast();
      gripperSubsystem.setOuttakePower(GripperConstants.gripperPower);
      holdMode = false;
    } else if (holdMode == true) {
      gripperSubsystem.setIntakePower(GripperConstants.gripperFeedforward);
      gripperSubsystem.brake();
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
