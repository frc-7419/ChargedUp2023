package frc.robot.subsystems.wrist;

import static frc.robot.constants.DeviceIDs.CanIds;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  private CANSparkMax wrist;
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(200, 130);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  /** Initializes the wrist motor and its encoder. */
  public WristSubsystem() {
    wrist = new CANSparkMax(CanIds.wristSpark.id, MotorType.kBrushless);
    zeroEncoder();
  }

  public void zeroEncoder(){
    wrist.getEncoder().setPosition(0);
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
    return wrist.getEncoder().getPosition() * 2 * Math.PI;
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
  /** Sets the motor power to 0, but doesn't brake. */
  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", getPosition());
  }
}
