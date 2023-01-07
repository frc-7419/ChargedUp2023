// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class LoaderSubsystem extends SubsystemBase {
  private TalonSRX loader;

  public LoaderSubsystem() {
    loader = new TalonSRX(CanIds.loaderVictor.id);
    loader.configFactoryDefault();
    loader.setInverted(true);
    loader.setSensorPhase(true);

    // enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
    // loaderSubsystem.getLoaderMotor().configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 15, 0.5));
    // loaderSubsystem.getLoaderMotor().configContinuousCurrentLimit(3);
    // loader.configContinuousCurrentLimit(10, 0);
    // loader.configPeakCurrentLimit(15, 0);
    // loader.configPeakCurrentDuration(100, 0);
    // loader.enableCurrentLimit(true);

    // loader.configVoltageCompSaturation(11);
    // loader.enableVoltageCompensation(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("SmartLoad Finished", false);

  }
  public TalonSRX getLoader() {
    return loader;
  }

  public void brake() {
    loader.setNeutralMode(NeutralMode.Brake);
  }
  public void coast() {
    loader.setNeutralMode(NeutralMode.Coast);
  }
  
  public void setPower(double power) {
    loader.set(ControlMode.PercentOutput, power);
  }
}
