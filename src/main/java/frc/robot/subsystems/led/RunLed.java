// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.LEDConstants;

/** This class sets the LED lights to our desired colors */
public class RunLed extends CommandBase {
  private LedSubsystem ledSubsystem;
  private XboxController joystick;
  /** Creates a new RunLed. */
  public RunLed(LedSubsystem ledSubsystem, XboxController joystick) {
    this.ledSubsystem = ledSubsystem;
    this.joystick = joystick;
    addRequirements(ledSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.startLed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /** Runs the LED */
  public void execute() {
    // ledSubsystem.setLEDColor(50, 0, 0);
    // ledSubsystem.rainbowLED(0);
    if (joystick.getAButtonPressed()) {
      ledSubsystem.setLEDColor(LEDConstants.yellowH, LEDConstants.yellowS, LEDConstants.yellowV);
    }
    if (joystick.getBButtonPressed()) {
      ledSubsystem.setLEDColor(LEDConstants.purpleH, LEDConstants.purpleS, LEDConstants.purpleV);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end (boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
