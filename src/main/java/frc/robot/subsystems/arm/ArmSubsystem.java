package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax mainArmMotor1;
  private CANSparkMax mainArmMotor2;
  private TalonSRX extendedArmMotor;
  private boolean homed;
  private double homePosition = 0;
  private DigitalInput magneticLimitSwitch;
  private PigeonIMU extendedGyro;

  public ArmSubsystem() {
    extendedArmMotor = new TalonSRX(CanIds.armExtended.id);

    mainArmMotor1 = new CANSparkMax(
      CanIds.armMain1.id, 
      MotorType.kBrushless); // ENCODER DOESNT WORK
    mainArmMotor2 = new CANSparkMax(
      CanIds.armMain2.id, 
      MotorType.kBrushless);

    mainArmMotor1.setInverted(false);
    mainArmMotor2.setInverted(true);

    magneticLimitSwitch = new DigitalInput(0); // port for now

    mainArmMotor2.getEncoder().setPositionConversionFactor(RobotConstants.mainArmGearRatio);
    extendedGyro = new PigeonIMU(0);
  }

  /**
   * 
   * @param Power set to both main and extended arms.
   */
  public void setAllPower(double power) {
    setMainPower(power);
    setExtendedPower(power);
  }

  /**
   * 
   * @param Power set to motors on the main arm.
   */
  public void setMainPower(double power) {
    mainArmMotor1.set(power);
    // mainMotor2.set(power);
  }

  /**
   * 
   * @param Power set to motor on the extended arm.
   */
  public void setExtendedPower(double power) {
    extendedArmMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * 
   * @return Returns the position of the main arms.
   */

  public double getMainPosition() { // THIS ENCODER DOES WORK
    return mainArmMotor2.getEncoder().getPosition() - homePosition;
  }

  /**
   * 
   * @return Returns the position of the extended arm.
   */
  public double getExtendedPosition() {
    return extendedArmMotor.getSelectedSensorPosition();
  }

  /**
   * @void Sets the homing position of the main arms.
   */
  public void home() {
    homePosition = getMainPosition();
  }

  /**
   * 
   * @return Returns the home position of the main arms.
   */
  public double getHomePosition() {
    return homePosition;
  }

  /**
   * 
   * @return Returns the yaw (angle of the extended arm).
   */
  public double getExtendedAngle() {
    double[] gyroInformation = new double[3];
    extendedGyro.getYawPitchRoll(gyroInformation);
    return gyroInformation[2];
  }

  public void coast() {
    coastMain();
    coastExtended();
  }

  public void coastMain() {
    mainArmMotor1.setIdleMode(IdleMode.kCoast);
    mainArmMotor2.setIdleMode(IdleMode.kCoast);
  }

  public void coastExtended() {
    extendedArmMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void brake() {
    brakeMain();
    brakeExtended();
  }

  public void brakeMain() {
    mainArmMotor1.setIdleMode(IdleMode.kBrake);
    mainArmMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void brakeExtended() {
    extendedArmMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    if (!magneticLimitSwitch.get() && !homed) {
      home();
      homed = true;
    }

    // outputting arm positions to smart dashboard and homing status

    SmartDashboard.putNumber("Arm Position", getMainPosition());
    SmartDashboard.putNumber("Arm Position (2)", getMainPosition());
    SmartDashboard.putNumber("Home Pos", homePosition);
    SmartDashboard.putBoolean("Arm Homed", homed);

    double[] gyroInformation = new double[3];
    extendedGyro.getYawPitchRoll(gyroInformation);

    // putting gyro values into smartdashboard
    SmartDashboard.putNumber("Extended Yaw", gyroInformation[0]);
    SmartDashboard.putNumber("Extended Pitch", gyroInformation[1]);
    SmartDashboard.putNumber("Extended Roll", gyroInformation[2]);
  }
}
