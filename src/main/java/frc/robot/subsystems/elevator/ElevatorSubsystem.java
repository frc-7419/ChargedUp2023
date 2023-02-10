// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;


public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax elevatorMaster;
  private CANSparkMax elevatorSlave;
  private RelativeEncoder encoder;

  public ElevatorSubsystem() {
    elevatorMaster = new CANSparkMax(CanIds.leftElevatorMotor.id, MotorType.kBrushless);
    elevatorSlave = new CANSparkMax(CanIds.rightElevatorMotor.id, MotorType.kBrushless);
    encoder = elevatorMaster.getEncoder();

    elevatorMaster.setInverted(true);
    elevatorSlave.follow(elevatorMaster, true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("position", getElevatorPosition());
    SmartDashboard.putNumber("Elevator Motor Output", getMotorOutputPercent());
  }

  public void setPower(double power) {
    elevatorMaster.set(power);
  }

  public void brake() {
    elevatorMaster.setIdleMode(IdleMode.kBrake);
  }

  public void coast() {
    elevatorMaster.setIdleMode(IdleMode.kCoast);
  }

  public double getElevatorPosition() {
    return encoder.getPosition();
  }

  public double getMotorOutputPercent() {
    return elevatorMaster.getAppliedOutput();
  }
}