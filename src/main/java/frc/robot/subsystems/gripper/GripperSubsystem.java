// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new gripper. */
  private CANSparkMax gripper;

  public boolean isHolding;

  public GripperSubsystem() {
    this.gripper = new CANSparkMax(DeviceIDs.CanIds.gripperNeo.id, MotorType.kBrushless);
    gripper.setSmartCurrentLimit(60, 30);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gripper velocity", getVelocity());
  }

  public void setPower(double power) {
    gripper.set(power);
  }

  public void stop() {
    gripper.set(0);
  }


  public void setIntakePower(double power) {
    gripper.set(power);
  }

  public void setOuttakePower(double power) {
    gripper.set(-power);
  }

  public void brake() {
    gripper.setIdleMode(IdleMode.kBrake);
  }

  public void coast() {
    gripper.setIdleMode(IdleMode.kCoast);
  }

  public double getVelocity() {
    return gripper.getEncoder().getVelocity();
  }
}
