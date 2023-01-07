// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class SetLEDColor extends CommandBase {
  private LEDSubsystem ledSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private DriveBaseSubsystem driveBaseSubsystem;

  private int rainbowFirstPixelHue = 0;

  private double limelightTolerance = 7.5;

  public SetLEDColor(LEDSubsystem ledSubsystem, LimelightSubsystem limelightSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
    this.ledSubsystem = ledSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.driveBaseSubsystem = driveBaseSubsystem;
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.startLed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ledSubsystem.rainbowLED(rainbowFirstPixelHue);
    // ledSubsystem.startLed();
    // rainbowFirstPixelHue += 3;
    // rainbowFirstPixelHue %= 180;
    if (limelightSubsystem.getTv() == 1.0 && Math.abs(limelightSubsystem.getTx()) <= limelightTolerance) {
      // green
      ledSubsystem.setLEDColor(0, 0, 255);
      // ledSubsystem.startLed();
    }
    else if (limelightSubsystem.getTv() == 1.0 && Math.abs(limelightSubsystem.getTx()) > limelightTolerance) {
      // yellow-ish white
      ledSubsystem.setLEDColor(255, 100, 150);
      // ledSubsystem.startLed();
    }
    else {
      // red
      ledSubsystem.setLEDColor(255, 0, 0);
      // ledSubsystem.startLed();
    }
    // int value = (int)(getAverageVelocity()/9000)*150+ 100;
    // ledSubsystem.rainbowLED1(rainbowFirstPixelHue, value);
    // rainbowFirstPixelHue += 3;
    // rainbowFirstPixelHue %= 180;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // purple
    ledSubsystem.setLEDColor(255, 255, 0);
    // ledSubsystem.setLED1Color(255, 255, 0);
    // ledSubsystem.stopLed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
