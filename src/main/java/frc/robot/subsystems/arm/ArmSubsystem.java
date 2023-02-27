package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax mainArmMotor1;
  private AnalogEncoder absoluteEncoder;
  private boolean homed;
  private double homePosition = 0;
  // private CANSparkMax mainArmMotor2;

  /** Constructs the extended arm and main arm subsystem corresponding to the arm mechanism. */
  public ArmSubsystem() {
    mainArmMotor1 =
        new CANSparkMax(CanIds.armMain1.id, MotorType.kBrushless); // ENCODER DOESNT WORK

    absoluteEncoder = new AnalogEncoder(3);

    // arbitrary cuz idk gearing stuff
    absoluteEncoder.setDistancePerRotation(10);
    absoluteEncoder.setPositionOffset(ArmConstants.armOffset);

    // mainArmMotor2 = new CANSparkMax(CanIds.armMain2.id, MotorType.kBrushless);

    // absoluteEncoder.setPositionConversionFactor(2 * RobotConstants.mainArmGearRatio / 4096);
    configureMotorControllers();
  }

  /**
   * Sets default inversions of left and right main motorcontrollers, and sets the conversion factor
   * for motorcontroller encoder
   */
  public void configureMotorControllers() {
    mainArmMotor1.setInverted(false);

    // mainArmMotor2.setInverted(true);
    // mainArmMotor2.getEncoder().setPositionConversionFactor(RobotConstants.mainArmGearRatio);
  }

  /**
   * Sets power to the motors on the main arm.
   *
   * @param power set to motors on the main arm.
   */
  public void setPower(double power) {
    mainArmMotor1.set(power);
    // mainMotor2.set(power);
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
    double position = absoluteEncoder.getAbsolutePosition() - absoluteEncoder.getPositionOffset();
    return position * 360;
  }

  /** Sets the home position variable to the current position of the main arm. */
  public void home() {
    homePosition = getPosition();
  }

  /**
   * Returns the home position of the main arms.
   *
   * @return The home position of the main arms.
   */
  public double getHomePosition() {
    return homePosition;
  }

  /** Sets arm motor to coast mode, allowing arm to freely move */
  public void coast() {
    mainArmMotor1.setIdleMode(IdleMode.kCoast);
    //  mainArmMotor2.setIdleMode(IdleMode.kCoast);
  }

  /** Sets arm motor to brake mode, stopping the arm's movement */
  public void brake() {
    mainArmMotor1.setIdleMode(IdleMode.kBrake);
    // mainArmMotor2.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // outputting arm positions to smart dashboard and homing status
    SmartDashboard.putNumber("Arm Position", getPosition());
    SmartDashboard.putNumber("Arm Angle", getAngle());
    // SmartDashboard.putNumber("Home Pos", homePosition);
    // SmartDashboard.putBoolean("Arm Homed", homed);
  }
}
