// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  CANSparkMax wrist;

  public WristSubsystem() {
    wrist = new CANSparkMax(CanIds.wristSpark.id, MotorType.kBrushless); //find canid
  }

  public void setPower(double power) {
    wrist.set(power);
  }

  public void coast() {
    wrist.setIdleMode(IdleMode.kCoast);
  }

  public void brake() {
    wrist.setIdleMode(IdleMode.kBrake);
  }

  public double getPosition() {
    return wrist.getEncoder().getPosition();
  }

  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {}
}
