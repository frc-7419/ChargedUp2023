// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import static frc.robot.Constants.CanIds.*;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  private Pigeon2 gyro;
  /** Creates a new PigeonSubsystem. */
  public GyroSubsystem() {
    this.gyro = new Pigeon2(pigeon.id);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("gyro yaw", getYaw());
    SmartDashboard.putNumber("gyro pitch", getPitch());
    SmartDashboard.putNumber("gyro roll", getRoll());
  }

  /**
   * Gyro yaw
   * @return yaw
   */
  public double getYaw() {
    return gyro.getYaw();
  }

  /**
   * Gyro Pitch
   * @return pitch
   */
  public double getPitch() {
    return gyro.getPitch();
  }

  /**
   * Gyro Roll
   * @return roll
   */
  public double getRoll() {
    return gyro.getRoll();
  }
}
