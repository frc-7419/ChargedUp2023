// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new gripper. */
  private TalonSRX gripper;

  public GripperSubsystem() {
    this.gripper = new TalonSRX(DeviceIDs.CanIds.gripperSRX.id);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    gripper.set(ControlMode.PercentOutput, power);
  }

  public void brake() {
    gripper.setNeutralMode(NeutralMode.Brake);
  }

  public void coast() {
    gripper.setNeutralMode(NeutralMode.Coast);
  }
}
