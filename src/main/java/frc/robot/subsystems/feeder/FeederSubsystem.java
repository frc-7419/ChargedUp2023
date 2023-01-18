// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class FeederSubsystem extends SubsystemBase {
  private TalonSRX feeder;

  public FeederSubsystem() {
    feeder = new TalonSRX(CanIds.feederTalon.id);
    feeder.configVoltageCompSaturation(11);
    feeder.enableVoltageCompensation(true);
  }

  public void setVoltage(double voltage) {
    feeder.set(ControlMode.PercentOutput, voltage / 11);
  }

  public void setPower(double power) {
    feeder.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
