// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.GripperConstants;
import frc.robot.constants.NodeConstants.PieceState;
import frc.robot.subsystems.led.LedSubsystem;

public class RunGripperWithJoystick extends CommandBase {
  private GripperSubsystem gripperSubsystem;
  private XboxController joystick;
  private LedSubsystem ledSubsystem;
  private double lastTimeStamp;
  private boolean holdMode = false;
  private boolean isIntaking = false;
  private boolean isOuttaking = false;
  private boolean isFast = true;

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
    if (joystick.getStartButtonPressed()) {
      isFast = !isFast;
    }
    double gripperOuttakePower = GripperConstants.gripperOuttakePower;
    if (isFast) {
      gripperOuttakePower = GripperConstants.gripperOuttakeFastPower;
    } else {
      gripperOuttakePower = GripperConstants.gripperOuttakePower;
    }
    // gripperSubsystem.isHolding = holdMode;

    // // if (!isOuttaking && joystick.getRightBumperPressed() && !holdMode) {
    // // isIntaking = !isIntaking;
    // // if (isIntaking) {
    // // holdMode = false;
    // // }
    // // }
    // // if (!isIntaking && joystick.getLeftBumperPressed()) {
    // // isOuttaking = !isOuttaking;
    // // if (isOuttaking) {
    // // holdMode = false;
    // // }
    // // }

    // if (joystick.getRightTriggerAxis() > 0.05) {
    // holdMode = false;
    // }
    if (joystick.getLeftBumper()) {
    isOuttaking = true;
    isIntaking = false;
    holdMode = false;
    } else if (joystick.getRightBumper()) {
    isOuttaking = false;
    isIntaking = true;
    } else {
    isOuttaking = false;
    isIntaking = false;
    }
    // if (holdMode) {
    // gripperSubsystem.setIntakePower(GripperConstants.gripperFeedforward);
    // gripperSubsystem.brake();
    // ledSubsystem.setLEDGreen();
    // } else if (isIntaking) {
    // gripperSubsystem.coast();
    // gripperSubsystem.setIntakePower(GripperConstants.gripperPower);
    // ledSubsystem.setLEDRed();
    // double currentTimeStamp = Timer.getFPGATimestamp();
    // double timePassed = currentTimeStamp - lastTimeStamp;
    // double velocityThreshold = SmartDashboard.getString("Piece State",
    // "CUBE").equals("CUBE") ? GripperConstants.cubeStallVelocityThreshold :
    // GripperConstants.coneStallVelocityThreshold;
    // boolean isStalling = gripperSubsystem.getVelocity() < velocityThreshold;
    // boolean didDelay = timePassed > 1;
    // if (isStalling && didDelay) {
    // // Hold mode won't be set to true unless we run it for 0.5 seconds to get the
    // // motor up to
    // // speed
    // holdMode = true;
    // isIntaking = false;
    // }
    // } else if (isOuttaking) {
    // gripperSubsystem.coast();
    // gripperSubsystem.setOuttakePower(gripperOuttakePower);
    // ledSubsystem.setLEDBlue();
    // } else {
    // gripperSubsystem.setPower(0);
    // lastTimeStamp = Timer.getFPGATimestamp();
    // gripperSubsystem.brake();
    // }
    if (isIntaking) {
      gripperSubsystem.coast();
      gripperSubsystem.setIntakePower(GripperConstants.gripperPower);
    } else if (isOuttaking) {
      gripperSubsystem.coast();
      gripperSubsystem.setOuttakePower(gripperOuttakePower);
    } else {
      gripperSubsystem.setPower(0);
      lastTimeStamp = Timer.getFPGATimestamp();
      gripperSubsystem.brake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
