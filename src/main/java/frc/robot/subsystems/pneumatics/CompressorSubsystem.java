// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private Compressor compressor;
  private boolean enabled = true;
  private boolean pressureSwitch;
  private double current;

  public CompressorSubsystem() {
    compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    compressor.enableDigital();
    pressureSwitch = compressor.getPressureSwitchValue();
    current = compressor.getCurrent();
  }

  public void start() { 
    compressor.enableDigital();
  }

  public void stop() {
    compressor.disable();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
