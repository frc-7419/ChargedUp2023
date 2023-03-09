package frc.robot.subsystems.wrist;

import static frc.robot.constants.DeviceIDs.CanIds;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  private CANSparkMax wrist;

  private AnalogEncoder absoluteEncoder;

  /** Initializes the wrist motor and its encoder. */
  public WristSubsystem() {
    wrist = new CANSparkMax(CanIds.wristSpark.id, MotorType.kBrushless); // find canid
    absoluteEncoder = new AnalogEncoder(DeviceIDs.SensorIds.wristAbsoluteEncoder.id);
  }

  /**
   * Sets the speed of the wrist motors to specified value.
   *
   * @param power from -1 to 1
   */
  public void setPower(double power) {
    wrist.set(power);
  }
  /** Sets the wrist to coast mode. */
  public void coast() {
    wrist.setIdleMode(IdleMode.kCoast);
  }
  /** Sets the wrist to brake mode. */
  public void brake() {
    wrist.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Uses the motor's integrated encoder to find the position of the arm.
   *
   * @return the encoder's measured position, in rotations
   */
  public double getPosition() {
    return absoluteEncoder.getAbsolutePosition() - WristConstants.wristOffset;
  }

  /** Sets the motor power to 0, but doesn't brake. */
  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", getPosition());
  }
}
