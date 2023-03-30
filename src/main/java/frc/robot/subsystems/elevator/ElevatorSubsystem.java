package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private final TalonFX elevatorMotor;

  // private AnalogEncoder absoluteEncoder;
  private ElevatorFeedforward elevatorFeedforward;

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(DeviceIDs.CanIds.mainElevatorMotor.id);
    elevatorMotor.setInverted(true);

    // elevatorMotor.configFactoryDefault();

    elevatorFeedforward = new ElevatorFeedforward(
        ElevatorConstants.elevatorKs,
        ElevatorConstants.elevatorKg,
        ElevatorConstants.elevatorKv);
    // elevatorMotor.set(ControlMode.PercentOutput, 0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.slot0.kP = ElevatorConstants.elevatorKp;
    config.slot0.kI = ElevatorConstants.elevatorKi;
    config.slot0.kD = ElevatorConstants.elevatorKd;
    config.slot0.kF = ElevatorConstants.elevatorFeedForward;
    // config.slot0.integralZone = // idk what this does
    config.slot0.closedLoopPeakOutput = ElevatorConstants.closedLoopPeakOutput;
    config.voltageCompSaturation = 11;

    config.motionAcceleration = 10000;
    config.motionCruiseVelocity = 10000;
    config.forwardSoftLimitThreshold = 330000;
    config.reverseSoftLimitThreshold = 5000;
    config.forwardSoftLimitEnable = true;
    config.reverseSoftLimitEnable = true;

    // // Soft Limits

    // config.forwardSoftLimitEnable = true;
    // config.forwardSoftLimitThreshold = 50000;
    // config.reverseSoftLimitEnable = true;
    // config.reverseSoftLimitThreshold = 0;


    elevatorMotor.configAllSettings(config);

    elevatorMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
    elevatorMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
  }

  public void zeroEncoder() {
    elevatorMotor.setSelectedSensorPosition(0);
  }

  public void setMotionMagic(double ticks) {
    double currentVelocity = elevatorMotor.getSelectedSensorVelocity(0);
    double calculatedFeedforward = elevatorFeedforward.calculate(currentVelocity);
    elevatorMotor.set(
        TalonFXControlMode.MotionMagic,
        ticks,
        DemandType.ArbitraryFeedForward,
        calculatedFeedforward);
  }

  /**
   * Sets the elevator power.
   *
   * @param percent [-1, 1] power set to the elevator
   */
  public void setPower(double percent) {
    elevatorMotor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Sets the desired goal state of the elevator.
   *
   * @param goalState the desired goal state of the elevator
   */
  public void setGoal(double goalState) {
    goal = new TrapezoidProfile.State(goalState, 0);
  }

  /**
   * Gets the desired goal state of the elevator.
   *
   * @return desired state of goal (Trapezoidal Profile State)
   */
  public TrapezoidProfile.State getGoal() {
    return goal;
  }

  /**
   * Sets the next setpoint for the elevator.
   *
   * @param nextSetpoint the next setpoint for the elevator
   */
  public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
    setpoint = nextSetpoint;
  }

  /**
   * Gets the elevator setpoint.
   *
   * @return The setpoint of the elevator
   */
  public TrapezoidProfile.State getSetpoint() {
    return setpoint;
  }

  /**
   * Gets the elevator constraints.
   *
   * @return the constraints (velocity and acceleration) for the trapezoidal
   *         profiling
   */
  public TrapezoidProfile.Constraints getConstraints() {
    return ElevatorConstants.constraints;
  }

  /**
   * Gets the elevator position.
   *
   * @return the current position of the elevator
   */
  // public double getElevatorPosition() {
  // return Units.rotationsToRadians(
  // absoluteEncoder.getAbsolutePosition() * ElevatorConstants.drumRadius / (2048
  // * 25));
  // }

  public double getElevatorIntegratedPosition() {
    return elevatorMotor.getSelectedSensorPosition()
        * (ElevatorConstants.metersPerRotation)
        / (2048 * ElevatorConstants.elevatorGearing);
  }

  public double getSetpointVelocity() {
    return elevatorMotor.getActiveTrajectoryVelocity();
  }

  public double getElevatorOutput() {
    return elevatorMotor.getMotorOutputPercent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "elevator position in ticks", elevatorMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("elevator position in meters", getElevatorIntegratedPosition());
    SmartDashboard.putNumber(
      "Elevator Power", elevatorMotor.getMotorOutputPercent());
  }

  /** Sets the elevator to brake mode. */
  public void brake() {
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  /** Sets the elevator to coast mode. */
  public void coast() {
    elevatorMotor.setNeutralMode(NeutralMode.Coast);
  }
}
