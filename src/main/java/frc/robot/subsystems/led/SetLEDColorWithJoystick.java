// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class SetLEDColorWithJoystick extends CommandBase {
  private XboxController joystick1;
  private XboxController joystick2;
  private LEDSubsystem ledSubsystem;
  private VisionSubsystem visionSubsystem;
  private DriveBaseSubsystem driveBaseSubsystem;
  
  private int rainbowFirstPixelHue = 0;
  // private int rainbowFirstPixelHue1 = 0;

  private double visionTolerance = 3.5;

  public SetLEDColorWithJoystick(LEDSubsystem ledSubsystem, VisionSubsystem visionSubsystem, DriveBaseSubsystem driveBaseSubsystem, XboxController joystick1, XboxController joystick2) {
    this.joystick1 = joystick1;
    this.joystick2 = joystick2;
    this.ledSubsystem = ledSubsystem;
    this.visionSubsystem = visionSubsystem;
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
    // ledSubsystem.setLEDColor(255, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // purple
    // ledSubsystem.setLEDColor(255, 255, 0);

    // rainbow
    ledSubsystem.rainbowLED(rainbowFirstPixelHue);
    ledSubsystem.startLed();
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
    // ledSubsystem.stopLed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
