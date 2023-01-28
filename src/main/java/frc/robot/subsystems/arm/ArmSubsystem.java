package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
  
  private CANSparkMax mainMotor1;
  private CANSparkMax mainMotor2;
  private TalonSRX extendedMotor;

  public ArmSubsystem() {
    extendedMotor = new TalonSRX(CanIds.armExtended.id);
    mainMotor1 = new CANSparkMax(CanIds.armMain1.id, MotorType.kBrushless);
    mainMotor2 = new CANSparkMax(CanIds.armMain2.id, MotorType.kBrushless);
    mainMotor1.getEncoder().setPositionConversionFactor(RobotConstants.mainArmGearRatio);
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

  public double getMainPosition() {
    return mainMotor1.getEncoder().getPosition(); //remember to gearratio it
  }
  
  public double getExtendedPosition() {
    return extendedMotor.getSelectedSensorPosition();
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

  @Override
  public void periodic() {}
}
