// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase {
  private DigitalInput beamBreakReceiver;
  private int detections = 0;
  private Long startTime;

  /** Creates a new BeamBreakSubsystem. */
  public BeamBreakSubsystem() {
    beamBreakReceiver = new DigitalInput(2);
    startTime = System.currentTimeMillis();
  }

  @Override
  public void periodic() {
    long endTime = System.currentTimeMillis();

    if (detections >= 0 && detections <= 2) {
      if (getBeamBreakActivated()) {
        detections++;
      } else {
        detections--;
      }

      if (endTime - startTime == 10000) {
        startTime = endTime;
        detections = 0;
      }
    }
  }

  public boolean getBeamBreakActivated() {
    return beamBreakReceiver.get(); // returns true if beam is activated
  }

  public DigitalInput getBeamBreakReceiver() {
    return beamBreakReceiver;
  }
}
