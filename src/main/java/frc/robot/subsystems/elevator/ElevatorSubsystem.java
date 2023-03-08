package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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

  // private final LinearSystem<N2, N1, N1> m_elevatorPlant =
  //     LinearSystemId.createElevatorSystem(
  //         DCMotor.getFalcon500(1),
  //         ElevatorConstants.carriageMass,
  //         Units.inchesToMeters(ElevatorConstants.drumRadius),
  //         ElevatorConstants.elevatorGearing);

  // private final KalmanFilter<N2, N1, N1> m_observer =
  //     new KalmanFilter<>(
  //         Nat.N2(),
  //         Nat.N1(),
  //         m_elevatorPlant,
  //         VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(40)),
  //         VecBuilder.fill(0.00001),
  //         0.020);

  // private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
  //     new LinearQuadraticRegulator<>(
  //         m_elevatorPlant,
  //         VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)),
  //         VecBuilder.fill(12.0),
  //         0.020);

  // private final LinearSystemLoop<N2, N1, N1> m_loop =
  //     new LinearSystemLoop<>(m_elevatorPlant, m_controller, m_observer, 12.0, 0.020);

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(DeviceIDs.CanIds.mainElevatorMotor.id);
    elevatorMotor.setSelectedSensorPosition(0);
    // elevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setPower(double percent) {
    // closedLoop = false;
    elevatorMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setGoal(double setpoint) {
    goal = new TrapezoidProfile.State(setpoint, 0);
  }

  public TrapezoidProfile.State getGoal() {
    return goal;
  }

  public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
    setpoint = nextSetpoint;
  }

  public TrapezoidProfile.State getSetpoint() {
    return setpoint;
  }

  public TrapezoidProfile.Constraints getConstraints() {
    return constraints;
  }

  public double getElevatorPosition() {
    return Units.rotationsToRadians(
        elevatorMotor.getSelectedSensorPosition() * ElevatorConstants.drumRadius / (2048 * 25));
  }

  @Override
  public void periodic() {
    // controllerPeriodic();
    SmartDashboard.putNumber("elevator position", getElevatorPosition());
  }

  public void brake() {
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void coast() {
    elevatorMotor.setNeutralMode(NeutralMode.Coast);
  }
}
