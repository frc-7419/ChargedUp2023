package frc.robot.subsystems.arm;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.SensorIds;
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

  /**
   * Constructs the extended arm and main arm subsystem corresponding to the arm mechanism.
   */
  public ArmSubsystem() {
    extendedArmMotor = new TalonSRX(CanIds.armExtended.id);

    mainArmMotor1 = new CANSparkMax(
      CanIds.armMain1.id, 
      MotorType.kBrushless); // ENCODER DOESNT WORK
    mainArmMotor2 = new CANSparkMax(
      CanIds.armMain2.id, 
      MotorType.kBrushless);
    
    magneticLimitSwitch = new DigitalInput(SensorIds.limitswitch.id); // port for now

    extendedGyro = new PigeonIMU(CanIds.extendedPigeon.id);
    configureMotorControllers();
  }

  /** 
   * Sets default inversions of left and right main motorcontrollers, 
   * and sets the conversion factor for motorcontroller encoder.
   * */
  public void configureMotorControllers() {
    mainArmMotor1.setInverted(false);
    mainArmMotor2.setInverted(true);

    mainArmMotor2.getEncoder().setPositionConversionFactor(RobotConstants.mainArmGearRatio);
  }

  /**
   * Sets power to both the main and extended arms.
   * @param power set to both main and extended arms.
   */
  public void setAllPower(double power) {
    setMainPower(power);
    setExtendedPower(power);
  }

  /**
   * Sets power to the motors on the main arm.
   * @param power set to motors on the main arm.
   */
  public void setMainPower(double power) {
    mainArmMotor1.set(power);
    // mainMotor2.set(power);
  }

  /**
   * Sets power to the motor on the extended arm.
   * @param power to motor on the extended arm.
   */
  public void setExtendedPower(double power) {
    extendedArmMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Returns the position of the main arm.
   * @return The position of the main arm, in units of rotations.
   */
  public double getMainPosition() { // THIS ENCODER DOES WORK
    return mainArmMotor2.getEncoder().getPosition() - homePosition;
  }

  /**
   * Returns the current position of the extended arm.
   * @return The position of the extended arm, in units of rotations.
   */
  public double getExtendedPosition() {
    return extendedArmMotor.getSelectedSensorPosition();
  }

  /** Sets the home position variable to the current position of the main arm. */
  public void home() {
    homePosition = getMainPosition();
  }

  /**
   * Returns the home position of the main arms.
   * @return The home position of the main arms.
   */
  public double getHomePosition() {
    return homePosition;
  }

  /**
   * Returns the yaw of the extended arm as measured by the arm's gyro.
   * @return The gyro's measured roll (angle of the extended arm).
   */
  public double getExtendedAngle() {
    double[] gyroInformation = new double[3];
    extendedGyro.getYawPitchRoll(gyroInformation);
    return gyroInformation[2];
  }

  /** Sets all arm motors to coast mode */
  public void coastAll() {
    coastMain();
    coastExtended();
  }

  /** Sets both main arm motors to coast mode, allowing main arm to freely move */
  public void coastMain() {
    mainArmMotor1.setIdleMode(IdleMode.kCoast);
    mainArmMotor2.setIdleMode(IdleMode.kCoast);
  }

  /** Sets extended arm motor to coast mode, allowing extended arm to freely move */
  public void coastExtended() {
    extendedArmMotor.setNeutralMode(NeutralMode.Coast);
  }

  /** Sets all arm motors to brake mode */
  public void brakeAll() {
    brakeMain();
    brakeExtended();
  }

  /** Sets both main arm motors to brake mode, stopping the main arm's movement */
  public void brakeMain() {
    mainArmMotor1.setIdleMode(IdleMode.kBrake);
    mainArmMotor2.setIdleMode(IdleMode.kBrake);
  }

  /** Sets extended arm's motor to brake mode, stopping the extended arm's movement */
  public void brakeExtended() {
    extendedArmMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Homes extended arm when magneticLimitSwitch is not detected and extended arm is not homed.
   * Puts arm positions, home position, if arm is homed to smart dashboard
   * Puts yaw, pitch, and roll of extended arm to smart dashboard
   */
  @Override
  public void periodic() {
    // when magneticLimitSwitch.get() returns true, it's detected
    // when magneticLimitSwitch is not detected and extended arm is not homed, 
    // home the extended arm and set variable homed to 0.
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
