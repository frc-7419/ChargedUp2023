package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

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
  private boolean homed;
  private double homePosition = 0;
  private DigitalInput magneticLimitSwitch;
  private PigeonIMU extendedGyro;

  /** Constructs the extended arm and main arm subsystem corresponding to the arm mechanism. */
  public ArmSubsystem() {
    mainArmMotor1 =
        new CANSparkMax(CanIds.armMain1.id, MotorType.kBrushless); // ENCODER DOESNT WORK
    mainArmMotor2 = new CANSparkMax(CanIds.armMain2.id, MotorType.kBrushless);

    magneticLimitSwitch = new DigitalInput(0); // port for now

    extendedGyro = new PigeonIMU(0);
    configureMotorControllers();
  }

  /**
   * Sets default inversions of left and right main motorcontrollers, and sets the conversion factor
   * for motorcontroller encoder
   */
  public void configureMotorControllers() {
    mainArmMotor1.setInverted(false);
    mainArmMotor2.setInverted(true);

    mainArmMotor2.getEncoder().setPositionConversionFactor(RobotConstants.mainArmGearRatio);
  }

  /**
   * Sets power to the motors on the main arm.
   *
   * @param power set to motors on the main arm.
   */
  public void setMainPower(double power) {
    mainArmMotor1.set(power);
    // mainMotor2.set(power);
  }
  /**
   * Returns the position of the main arm, relative to the arm's home position.
   *
   * @return The position of the main arm, in units of rotations.
   */
  public double getMainPosition() {
    return mainArmMotor2.getEncoder().getPosition()
        - homePosition; // main arm motor 2's encoder works
  }

  /** Sets the home position variable to the current position of the main arm. */
  public void home() {
    homePosition = getMainPosition();
  }

  /**
   * Returns the home position of the main arms.
   *
   * @return The home position of the main arms.
   */
  public double getHomePosition() {
    return homePosition;
  }

  /** Sets all arm motors to coast mode */
  public void coastAll() {
    coastMain();
  }

  /** Sets both main arm motors to coast mode, allowing main arm to freely move */
  public void coastMain() {
    mainArmMotor1.setIdleMode(IdleMode.kCoast);
    mainArmMotor2.setIdleMode(IdleMode.kCoast);
  }

  /** Sets all arm motors to brake mode */
  public void brakeAll() {
    brakeMain();
  }

  /** Sets both main arm motors to brake mode, stopping the main arm's movement */
  public void brakeMain() {
    mainArmMotor1.setIdleMode(IdleMode.kBrake);
    mainArmMotor2.setIdleMode(IdleMode.kBrake);
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
