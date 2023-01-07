// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class ArmsSubsystem extends SubsystemBase {
  //one motor controls both arms
  private CANSparkMax leftArm;
  private CANSparkMax rightArm;
  private DigitalInput limitSwitch;
  private boolean homed = false;
  private RelativeEncoder encoder;
  private double homePos = 0;
  
  public ArmsSubsystem() {
    this.leftArm = new CANSparkMax(CanIds.armSpark1.id, MotorType.kBrushless);
    this.rightArm = new CANSparkMax(CanIds.armSpark2.id, MotorType.kBrushless);
    this.limitSwitch = new DigitalInput(0);
    this.encoder = leftArm.getEncoder();
    rightArm.setInverted(true);
    
    leftArm.enableVoltageCompensation(11);
    rightArm.enableVoltageCompensation(11);
    // brake();
  }

  @Override
  public void periodic() {
    if (!homed && !limitSwitch.get()) {
      zero();
    }
    SmartDashboard.putBoolean("armHomed", homed);
    SmartDashboard.putNumber("armHomePos", homePos);
    SmartDashboard.putNumber("armPos", getPosition());
    SmartDashboard.putBoolean("limitSwitch", !limitSwitch.get());
    // SmartDashboard.putNumber("Arms Motor Output", leftArm.getAppliedOutput());
    // SmartDashboard.putNumber("ArmCurrent", getCurrent());
  }

  public void zero() {
    homePos = encoder.getPosition();
    homed = true;
  }

  public double getPosition() {
    return encoder.getPosition() - homePos;
  }

  public void setPower(double power) {
    leftArm.set(power);
    rightArm.set(power);
  }
  public void flash() {
    leftArm.burnFlash();
    rightArm.burnFlash();
  }
  public void brake() {
    leftArm.setIdleMode(IdleMode.kBrake);
    rightArm.setIdleMode(IdleMode.kBrake);
    flash();
  }
  public void coast() {
    leftArm.setIdleMode(IdleMode.kCoast);
    rightArm.setIdleMode(IdleMode.kCoast);
    flash();
  }

  public double getCurrent() {
    return leftArm.getOutputCurrent();
  }
  
  // public IdleMode getIdleMode() {
  //   return leftArm.getIdleMode();
  // }
}