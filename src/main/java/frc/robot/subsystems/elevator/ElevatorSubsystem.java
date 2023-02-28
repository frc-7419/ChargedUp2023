package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0.0, 0.0);
  private boolean closedLoop = false;

  private final TalonFX elevatorMotor;

  private final LinearSystem<N2, N1, N1> m_elevatorPlant =
      LinearSystemId.createElevatorSystem(
          DCMotor.getFalcon500(1),
          ElevatorConstants.carriageMass,
          1,
          ElevatorConstants.elevatorGearing);

  private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          m_elevatorPlant,
          VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(40)),
          VecBuilder.fill(0.01),
          0.020);

  private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_elevatorPlant,
          VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)),
          VecBuilder.fill(12.0),
          0.020);

  private final LinearSystemLoop<N2, N1, N1> m_loop =
      new LinearSystemLoop<>(m_elevatorPlant, m_controller, m_observer, 12.0, 0.020);

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(DeviceIDs.CanIds.mainElevatorMotor.id);
    elevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public void reset() {
    m_loop.reset(VecBuilder.fill(getElevatorPosition(), getRate()));
    m_lastProfiledReference = new TrapezoidProfile.State(getElevatorPosition(), getRate());
  }

  public void setSpeed(double percent) {
    closedLoop = false;
    elevatorMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setPower(double percent) {
    closedLoop = false;
    elevatorMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setGoal(double positionMeters) {
    if (!closedLoop) {
      closedLoop = true;
      reset();
    }

    if (positionMeters > 0.0) {
      goal = new TrapezoidProfile.State(positionMeters, 0.0);
    } else {
      goal = new TrapezoidProfile.State(0.0, 0.0);
    }
  }

  public double getElevatorPosition() {
    return Units.rotationsToRadians(
        elevatorMotor.getSelectedSensorPosition() * ElevatorConstants.drumRadius);
  }

  public double getRate() {
    return Units.rotationsPerMinuteToRadiansPerSecond(
        elevatorMotor.getSelectedSensorVelocity() * ElevatorConstants.drumRadius);
  }

  public void controllerPeriodic() {
    if (closedLoop) {
      m_lastProfiledReference =
          (new TrapezoidProfile(ElevatorConstants.m_constraints, goal, m_lastProfiledReference))
              .calculate(0.020);
      m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

      m_loop.correct(VecBuilder.fill(getElevatorPosition()));

      m_loop.predict(0.020);

      double nextVoltage = m_loop.getU(0);
      elevatorMotor.set(ControlMode.PercentOutput, nextVoltage / 11);
    }
  }

  @Override
  public void periodic() {
    controllerPeriodic();
    SmartDashboard.putNumber("elevator position", getElevatorPosition());
  }

  public void brake() {
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void coast() {
    elevatorMotor.setNeutralMode(NeutralMode.Coast);
  }
}
