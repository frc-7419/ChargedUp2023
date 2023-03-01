package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.DeviceIDs;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax mainArmMotor1;
  private AnalogEncoder absoluteEncoder;

  /** Constructs the extended arm and main arm subsystem corresponding to the arm mechanism. */
  public ArmSubsystem() {
    mainArmMotor1 =
        new CANSparkMax(DeviceIDs.CanIds.armMain1.id, MotorType.kBrushless); // ENCODER DOESNT WORK

    absoluteEncoder = new AnalogEncoder(3);

    configureEncoder();
    configureMotorControllers();
  }

  /**
   * Sets default inversions of left and right main motorcontrollers, and sets the conversion factor
   * for motorcontroller encoder
   */
  public void configureMotorControllers() {
    mainArmMotor1.setInverted(false);
    // mainArmMotor2.getEncoder().setPositionConversionFactor(RobotConstants.mainArmGearRatio);
  }

  public void configureEncoder() {
    absoluteEncoder.setPositionOffset(ArmConstants.armOffset);
  }

  /**
   * Sets power to the motors on the main arm.
   *
   * @param power set to motors on the main arm.
   */
  public void setPower(double power) {
    mainArmMotor1.set(power);
  }

  /**
   * Returns the position of the main arm, relative to the arm's home position.
   *
   * @return The position of the main arm, in units of rotations.
   */
  public double getPosition() {
    return absoluteEncoder.getAbsolutePosition();
  }

  public double getAngle() {
    return getPosition() * 360;
  }

  /** Sets arm motor to coast mode, allowing arm to freely move */
  public void coast() {
    mainArmMotor1.setIdleMode(IdleMode.kCoast);
  }

  /** Sets arm motor to brake mode, stopping the arm's movement */
  public void brake() {
    mainArmMotor1.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // outputting arm positions to smart dashboard and homing status
    SmartDashboard.putNumber("Arm Position", getPosition());
    SmartDashboard.putNumber("Arm Angle", getAngle());
  }
}
