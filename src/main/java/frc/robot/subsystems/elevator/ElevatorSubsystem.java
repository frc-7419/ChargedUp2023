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
  private CANSparkMax elevatorFollower;
  private RelativeEncoder encoder;

  /** Constructs an elevator subsystem */
  public ElevatorSubsystem() {
    elevatorMaster = new CANSparkMax(CanIds.leftElevatorMotor.id, MotorType.kBrushless);
    elevatorFollower = new CANSparkMax(CanIds.rightElevatorMotor.id, MotorType.kBrushless);
    encoder = elevatorMaster.getEncoder();

    elevatorMaster.setInverted(true);
    elevatorFollower.follow(elevatorMaster, true);
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
    elevatorMaster.set(power);
  }

  /** Set brake mode */
  public void brake() {
    elevatorMaster.setIdleMode(IdleMode.kBrake);
  }

  /** Set coast mode */
  public void coast() {
    elevatorMaster.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Get elevator position
   *
   * @return position in rotations
   */
  public double getElevatorPosition() {
    return encoder.getPosition();
  }

  /**
   * Get elevator output
   *
   * @return output between -1.0 and 1.0
   */
  public double getMotorOutputPercent() {
    return elevatorMaster.getAppliedOutput();
  }
}
