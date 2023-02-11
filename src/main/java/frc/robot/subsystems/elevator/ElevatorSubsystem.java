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
  private CANSparkMax elevatorMain;
  private CANSparkMax elevatorFollow;
  private RelativeEncoder encoder;

  /**
   * Constructs an elevator subsystem
   */
  public ElevatorSubsystem() {
    elevatorMain = new CANSparkMax(CanIds.leftElevatorMotor.id, MotorType.kBrushless);
    elevatorFollow = new CANSparkMax(CanIds.rightElevatorMotor.id, MotorType.kBrushless);
    encoder = elevatorMain.getEncoder();

    elevatorMain.setInverted(true);
    elevatorFollow.follow(elevatorMain, true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("position", getElevatorPosition());
    SmartDashboard.putNumber("Elevator Motor Output", getMotorOutputPercent());
  }

  /**
   * Sets the power of the motor
   * @param power input power between -1 and 1
   */
  public void setPower(double power) {
    elevatorMain.set(power);
  }

  /**
   * Set brake mode
   */
  public void brake() {
    elevatorMain.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Set coast mode
   */
  public void coast() {
    elevatorMain.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Get elevator position
   * @return position in rotations
   */
  public double getElevatorPosition() {
    return encoder.getPosition();
  }

  /**
   * Get elevator output
   * @return output between -1.0 and 1.0
   */
  public double getMotorOutputPercent() {
    return elevatorMain.getAppliedOutput();
  }
}