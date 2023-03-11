// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.GripperConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.led.LedSubsystem;

public class RunGripperWithJoystick extends CommandBase {
  private GripperSubsystem gripperSubsystem;
  private XboxController joystick;
  private LedSubsystem ledSubsystem;
  private double lastTimeStamp;
  private Boolean holdMode = false;

  public RunGripperWithJoystick(
      GripperSubsystem gripperSubsystem, XboxController joystick, LedSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripperSubsystem = gripperSubsystem;
    this.joystick = joystick;
    this.ledSubsystem = ledSubsystem;
    addRequirements(gripperSubsystem, ledSubsystem);
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

    boolean leftTriggerActivated = joystick.getLeftTriggerAxis() >= RobotConstants.joystickDeadZone;
    boolean rightTriggerActivated = joystick.getRightTriggerAxis() >= RobotConstants.joystickDeadZone;

    if (rightTriggerActivated && !holdMode) {
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
      }
    } else if (leftTriggerActivated) {
      gripperSubsystem.coast();
      gripperSubsystem.setOuttakePower(GripperConstants.gripperPower);
      ledSubsystem.setLEDBlue();
      holdMode = false;
    } else if (holdMode) {
      gripperSubsystem.setIntakePower(GripperConstants.gripperFeedforward);
      gripperSubsystem.brake();
      ledSubsystem.setLEDGreen();
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
