// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new gripper. */
  private CANSparkMax gripper;

  public GripperSubsystem() {
    this.gripper = new CANSparkMax(CanIds.gripperSpark.id, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power){
    gripper.set(power);
  }

  public void brake() {
    gripper.setIdleMode(IdleMode.kBrake);
  }
  public void coast() {
    gripper.setIdleMode(IdleMode.kCoast);
  }
}
