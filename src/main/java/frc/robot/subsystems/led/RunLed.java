// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.LEDConstants;

/** This class sets the LED lights to our desired colors */
public class RunLed extends CommandBase {
  private LedSubsystem ledSubsystem;
  /** Creates a new RunLed. */
  public RunLed(LedSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
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
    ledSubsystem.setLEDColor(LEDConstants.yellowH, LEDConstants.yellowS, LEDConstants.yellowV);
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
