package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(300, 150);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private final TalonFX elevatorMotor;

  private AnalogEncoder absoluteEncoder;

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(DeviceIDs.CanIds.mainElevatorMotor.id);
    elevatorMotor.setSelectedSensorPosition(0);

    absoluteEncoder = new AnalogEncoder(2);
    // elevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Sets the elevator power.
   * @param power [-1, 1] power set to the elevator
   */
  public void setPower(double percent) {
    elevatorMotor.set(ControlMode.PercentOutput, percent);
  }

  
  /**
   * Sets the desired goal state of the elevator.
   * @param goal the desired goal state of the elevator
   */
  public void setGoal(double goalState) {
    goal = new TrapezoidProfile.State(goalState, 0);
  }

  /**
   * Gets the desired gola state of the elevator.
   * @return desired state of goal (Trapezoidal Profile State) 
   */
  public TrapezoidProfile.State getGoal() {
    return goal;
  }
  
  /**
   * Sets the next setpoint for the elevator.
   * @param nextSetpoint the next setpoint for the elevator
   */
  public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
    setpoint = nextSetpoint;
  }

  /**
   * Gets the elevator setpoint.
   * @return The setpoint of the elevator
   */
  public TrapezoidProfile.State getSetpoint() {
    return setpoint;
  }

  /**
   * Gets the elevator constraints.
   * @return the constraints (velocity and acceleration) for the trapezoidal profiling
   */
  public TrapezoidProfile.Constraints getConstraints() {
    return constraints;
  }

  /**
   * Gets the elevator position.
   * @return the current position of the elevator
   */
  public double getElevatorPosition() {
    return Units.rotationsToRadians(
        absoluteEncoder.getAbsolutePosition() * ElevatorConstants.drumRadius / (2048 * 25));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator position", getElevatorPosition());
  }
  
  /**
   * Sets the elevator to brake mode.
   */
  public void brake() {
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Sets the elevator to coast mode.
   */
  public void coast() {
    elevatorMotor.setNeutralMode(NeutralMode.Coast);
  }
}
