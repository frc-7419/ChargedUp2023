package frc.robot.subsystems.arm;

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
import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
  
  private CANSparkMax mainMotor1;
  private CANSparkMax mainMotor2;
  private TalonSRX extendedMotor;
  private boolean homed;
  private double homePosition = 0;
  private DigitalInput magLimSwitch;
  private PigeonIMU extendedGyro;

  public ArmSubsystem() {
    extendedMotor = new TalonSRX(CanIds.armExtended.id);
    mainMotor1 = new CANSparkMax(CanIds.armMain1.id, MotorType.kBrushless); // ENCODER DOESNT WORK
    mainMotor2 = new CANSparkMax(CanIds.armMain2.id, MotorType.kBrushless);
    mainMotor1.setInverted(false);
    mainMotor2.setInverted(true);
    magLimSwitch = new DigitalInput(0); // port for now
    mainMotor2.getEncoder().setPositionConversionFactor(RobotConstants.mainArmGearRatio);
    extendedGyro = new PigeonIMU(0);
  }

  public void setAllPower(double power) {
    setMainPower(power);
    setExtendedPower(power);
  }

  public void setMainPower(double power) {
    mainMotor1.set(power);
    mainMotor2.set(power);
  }

  public void setExtendedPower(double power) {
    extendedMotor.set(ControlMode.PercentOutput, power);
  }

  public double getMainPosition() { // THIS ENCODER DOESNT WORK
    return mainMotor1.getEncoder().getPosition() - homePosition;
  }

  public double getMain2Position() { // THIS ENCODER DOES WORK
    return mainMotor2.getEncoder().getPosition() - homePosition;
  }
  
  public double getExtendedPosition() {
    return extendedMotor.getSelectedSensorPosition();
  }

  public double getExtendedAngle() {
    double[] ypr = new double[3];
    extendedGyro.getYawPitchRoll(ypr);
    return ypr[2];
  }
  
  public void coast() {
    coastMain();
    coastExtended();
  }

  public void coastMain() {
    mainMotor1.setIdleMode(IdleMode.kCoast);
    mainMotor2.setIdleMode(IdleMode.kCoast);
  }

  public void coastExtended() {
    extendedMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void brake() {
    brakeMain();
    brakeExtended();
  }

  public void brakeMain() {
    mainMotor1.setIdleMode(IdleMode.kBrake);
    mainMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void brakeExtended() {
    extendedMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void home() {
    homePosition = getMainPosition();
  }

  public double getHomePosition() {
    return homePosition;
  }

  @Override
  public void periodic() {
    if(!magLimSwitch.get() && !homed) {
      home();
      homed = true;
    }
    SmartDashboard.putNumber("Arm Position", getMainPosition());
    SmartDashboard.putNumber("Arm position (2)", getMain2Position());
    SmartDashboard.putNumber("Home Pos", homePosition);
    SmartDashboard.putBoolean("Arm Homed", homed);
    SmartDashboard.putBoolean("Arm Cool?", !magLimSwitch.get());
    double[] ypr = new double[3];
    extendedGyro.getYawPitchRoll(ypr);
    SmartDashboard.putNumber("Extended Yaw", ypr[0]);
    SmartDashboard.putNumber("Extended Pitch", ypr[1]);
    SmartDashboard.putNumber("Extended Roll", ypr[2]);
  }
}
