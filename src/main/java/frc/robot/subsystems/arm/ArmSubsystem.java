package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.DeviceIDs;

public class ArmSubsystem extends SubsystemBase {

  private TalonFX armMotor;
  private DutyCycleEncoder absoluteEncoder;

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(300, 150);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  /** Constructs the extended arm and main arm subsystem corresponding to the arm mechanism. */
  public ArmSubsystem() {
    armMotor = new TalonFX(DeviceIDs.CanIds.armFalcon.id);
    absoluteEncoder = new DutyCycleEncoder(DeviceIDs.SensorIds.armAbsoluteEncoder.id);
    configureMotorControllers();
  }

  /**
   * Sets default inversions of left and right main motorcontrollers, and sets the conversion factor
   * for motorcontroller encoder
   */
  public void configureMotorControllers() {
    armMotor.setInverted(false);
  }

  /**
   * Sets the desired goal state of the arm.
   *
   * @param goalState the desired goal state of the arm
   */
  public void setGoal(double goalState) {
    goal = new TrapezoidProfile.State(goalState, 0);
  }

  /**
   * Gets the desired goal state of the arm.
   *
   * @return desired state of goal (Trapezoidal Profile State)
   */
  public TrapezoidProfile.State getGoal() {
    return goal;
  }

  /**
   * Sets the next setpoint for the arm.
   *
   * @param nextSetpoint the next setpoint for the arm
   */
  public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
    setpoint = nextSetpoint;
  }

  /**
   * Gets the arm setpoint.
   *
   * @return The setpoint of the arm
   */
  public TrapezoidProfile.State getSetpoint() {
    return setpoint;
  }

  /**
   * Gets the arm constraints.
   *
   * @return the constraints (velocity and acceleration) for the trapezoidal profiling
   */
  public TrapezoidProfile.Constraints getConstraints() {
    return constraints;
  }

  /**
   * Sets position offset for the absolute encoder (i.e. after the offset, the absolute position
   * reading will be 0 when the arm is parallel to the ground)
   */
  public void configureEncoder() {
    absoluteEncoder.setPositionOffset(ArmConstants.armOffset);
  }

  /**
   * Sets power to the motors on the main arm.
   *
   * @param power set to motors on the main arm.
   */
  public void setPower(double power) {
    armMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Returns the position of the main arm, relative to the arm's home position.
   *
   * @return The position of the main arm, in units of rotations.
   */
  public double getPosition() {
    return absoluteEncoder.getAbsolutePosition() - ArmConstants.armOffset; // 0 should = horizontal
  }

  /**
   * Gets the arm rotation in degrees.
   *
   * @return the position in degrees.
   */
  public double getAngle() {
    return getPosition() * 360;
  }

  /** Sets arm motor to coast mode, allowing arm to freely move */
  public void coast() {
    armMotor.setNeutralMode(NeutralMode.Coast);
  }

  /** Sets arm motor to brake mode, stopping the arm's movement */
  public void brake() {
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // outputting arm positions to smart dashboard and homing status
    SmartDashboard.putNumber("Arm Position", getPosition());
    SmartDashboard.putNumber("Arm Angle", getAngle());
  }
}
