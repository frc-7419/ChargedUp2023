package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.DeviceIDs;

public class ArmSubsystem extends SubsystemBase {

  private TalonFX armMotor;
  private DutyCycleEncoder absoluteEncoder;

  /** Constructs the extended arm and main arm subsystem corresponding to the arm mechanism. */
  public ArmSubsystem() {
    armMotor = new TalonFX(DeviceIDs.CanIds.armFalcon.id);
    absoluteEncoder = new DutyCycleEncoder(DeviceIDs.SensorIds.armAbsoluteEncoder.id);
    configureMotorControllers();
  }

  /**
   * Sets default inversions of left and right main motorcontrollers, and sets the conversion factor
   * for motorcontroller encoder
   */
  public void configureMotorControllers() {
    armMotor.setInverted(false);
  }

  /**
   * Sets power to the motors on the main arm.
   *
   * @param power set to motors on the main arm.
   */
  public void setPower(double power) {
    armMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Returns the position of the main arm, relative to the arm's home position.
   *
   * @return The position of the main arm, in units of rotations.
   */
  public double getPosition() {
    return absoluteEncoder.getAbsolutePosition() - ArmConstants.armOffset;
  }

  public double getAngle() {
    return getPosition() * 360;
  }

  /** Sets arm motor to coast mode, allowing arm to freely move */
  public void coast() {
    armMotor.setNeutralMode(NeutralMode.Coast);
  }

  /** Sets arm motor to brake mode, stopping the arm's movement */
  public void brake() {
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // outputting arm positions to smart dashboard and homing status
    SmartDashboard.putNumber("Arm Position", getPosition());
    SmartDashboard.putNumber("Arm Angle", getAngle());
  }
}
