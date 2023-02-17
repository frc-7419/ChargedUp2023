// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor;

  /** Constructs an elevator subsystem */
  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(Constants.CanIds.mainElevatorMotor.id);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("position", getElevatorPosition());
    SmartDashboard.putNumber("Elevator Motor Output", getMotorOutputPercent());
  }

  /**
   * Sets the power of the motor
   *
   * @param power input power between -1 and 1
   */
  public void setPower(double power) {
    elevatorMotor.set(ControlMode.PercentOutput, power);
  }

  /** Set brake mode */
  public void brake() {
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  /** Set coast mode */
  public void coast() {
    elevatorMotor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Get elevator position
   *
   * @return position in rotations
   */
  public double getElevatorPosition() {
    return elevatorMotor.getSelectedSensorPosition();
  }

  /**
   * Get elevator output
   *
   * @return output between -1.0 and 1.0
   */
  public double getMotorOutputPercent() {
    return elevatorMotor.getMotorOutputPercent();
  }
}