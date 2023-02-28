// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;

public class BeamBreakSubsystem extends SubsystemBase {
  private DigitalInput beamBreakReceiver;

  /** Creates a new BeamBreakSubsystem. */
  public BeamBreakSubsystem() {
    beamBreakReceiver = new DigitalInput(DeviceIDs.SensorIds.beambreak.id);
  }

  @Override
  public void periodic() {}

  /**
   * Returns true if the beam break is open and false if the beam break is blocked.
   *
   * @return True if the beam break is open(unobstructed) and false if the beam break is blocked
   */
  public boolean getBeamBreakActivated() {
    return beamBreakReceiver.get();
  }

  /**
   * Return DigitalInput beam break receiver sensor.
   *
   * @return DigitalInput beam break receiver sensor
   */
  public DigitalInput getBeamBreakReceiver() {
    return beamBreakReceiver;
  }
}
