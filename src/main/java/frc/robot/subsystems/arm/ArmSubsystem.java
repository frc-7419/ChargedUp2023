package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

  public ArmSubsystem() {
    extendedMotor = new TalonSRX(CanIds.armExtended.id);
    mainMotor1 = new CANSparkMax(CanIds.armMain1.id, MotorType.kBrushless);
    mainMotor2 = new CANSparkMax(CanIds.armMain2.id, MotorType.kBrushless);
    magLimSwitch = new DigitalInput(0); // port for now
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
    return mainMotor1.getEncoder().getPosition() - homePosition; //remember to gearratio it
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
    SmartDashboard.putNumber("Home Pos", homePosition);
    SmartDashboard.putBoolean("Arm Homed", homed);
  }
}
