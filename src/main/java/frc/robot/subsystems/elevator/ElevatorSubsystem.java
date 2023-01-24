// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team7419.TalonFuncs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorLeft;
  private TalonFX elevatorRight;

  public ElevatorSubsystem() {
    elevatorLeft = new TalonFX(CanIds.leftElevatorFalcon.id);
    elevatorRight = new TalonFX(CanIds.rightElevatorFalcon.id);

    elevatorRight.follow(elevatorLeft);

    elevatorLeft.setInverted(true);
    elevatorRight.setInverted(InvertType.OpposeMaster);

    elevatorLeft.configVoltageCompSaturation(11);
    elevatorLeft.enableVoltageCompensation(true);
    elevatorRight.configVoltageCompSaturation(11);
    elevatorRight.enableVoltageCompensation(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("position", elevatorLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Motor Output", elevatorLeft.getMotorOutputPercent());
  }

  public double getOutput() {
    return elevatorLeft.getMotorOutputPercent();
  }

  public void setPower(double power) {
    elevatorLeft.set(ControlMode.PercentOutput, power);
    elevatorRight.set(ControlMode.PercentOutput, power);
  }

  public void brake() {
    elevatorLeft.setNeutralMode(NeutralMode.Brake);
    elevatorRight.setNeutralMode(NeutralMode.Brake);
  }

  public void coast() {
    elevatorLeft.setNeutralMode(NeutralMode.Coast);
    elevatorRight.setNeutralMode(NeutralMode.Coast);
  }

  public void setPIDFConstants(double kP, double kI, double kD, double kF) {
    TalonFuncs.setPIDFConstants(0, elevatorLeft, kP, kI, kD, kF);
  }

  public double getElevatorPosition() {
    return elevatorLeft.getSelectedSensorPosition();
  }

  public TalonFX getElevatorLeft() {
    return elevatorLeft;
  }

  public TalonFX getElevatorRight() {
    return elevatorRight;
  }
}