package frc.robot.subsystems.gyro;

import frc.robot.constants.DeviceIDs;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  private Pigeon2 gyro;

  /** Instatiates the pigeon and resets its yaw to 0 degrees. */
  public GyroSubsystem() {
    this.gyro = new Pigeon2(DeviceIDs.CanIds.pigeon.id);
    gyro.setYaw(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyro yaw", getYaw());
    SmartDashboard.putNumber("gyro pitch", getPitch());
    SmartDashboard.putNumber("gyro roll", getRoll());
  }

  /**
   * Returns gyro yaw, in degrees
   *
   * @return The pigeon's measured yaw in degrees
   */
  public double getYaw() {
    return gyro.getYaw();
  }

  /**
   * Returns gyro pitch, in degrees
   *
   * @return The pigeon's measured pitch in degrees
   */
  public double getPitch() {
    return gyro.getPitch();
  }

  /**
   * Returns gyro roll, in degrees
   *
   * @return The pigeon's measured roll in degrees
   */
  public double getRoll() {
    return gyro.getRoll();
  }
}
