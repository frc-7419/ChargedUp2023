package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.RobotConstants;

public class ArmSubsystem extends SubsystemBase {

  private TalonFX armMotor;
  private DutyCycleEncoder absoluteEncoder;
  private double zeroAngleOffset = 60;
  private double offsetInTicks = 34693;
  // private Encoder relativeEncoder;
  private double offset = 0;
  private double ks = ArmConstants.withoutConeks;
  private double kg = ArmConstants.withoutConekg;
  private double kv = ArmConstants.withoutConekv;
  private double ka = ArmConstants.withoutConeka;

  private final TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private ArmFeedforward armFeedforward = ArmConstants.armFeedforward;


  /** Constructs the extended arm and main arm subsystem corresponding to the arm mechanism. */
  public ArmSubsystem() {
    constraints  = new TrapezoidProfile.Constraints(ArmConstants.maxVelocity,
    ArmConstants.maxAcceleration);
    armMotor = new TalonFX(DeviceIDs.CanIds.armFalcon.id);
    absoluteEncoder = new DutyCycleEncoder(DeviceIDs.SensorIds.armAbsoluteEncoder.id);
    configureMotorControllers();

    offset = absoluteEncoder.getAbsolutePosition();
    armMotor.setSelectedSensorPosition(offset * ArmConstants.armGearing * 2048);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.slot0.kP = ArmConstants.armKp;
    config.slot0.kI = ArmConstants.armKi;
    config.slot0.kD = ArmConstants.armKd;
    config.slot0.kF = armFeedforward.calculate(Units.degreesToRadians(getAngle()), 0);
    // config.slot0.integralZone = // idk what this does
    config.slot0.closedLoopPeakOutput = ArmConstants.closedLoopPeakOutput;
    config.voltageCompSaturation = RobotConstants.voltageCompSaturation;
    config.motionAcceleration = 10000;
    config.motionCruiseVelocity = 10000;

    config.forwardSoftLimitThreshold = 335000;

    config.reverseSoftLimitThreshold = 5000;
    config.forwardSoftLimitEnable = true;
    config.reverseSoftLimitEnable = true;
  }

  /**
   * Sets default inversions of left and right main motorcontrollers, and sets the
   * conversion factor
   * for motorcontroller encoder
   */
  public void configureMotorControllers() {
    armMotor.setInverted(false);
  }

  public void setMotionMagic(double ticks) {
    double currentPositionRadians = Units.degreesToRadians(getAngle());
    double currentVelocity = armMotor.getSelectedSensorVelocity(0);
    double calculatedFeedforward = armFeedforward.calculate(currentPositionRadians,currentVelocity);
    armMotor.set(
        TalonFXControlMode.MotionMagic,
        ticks,
        DemandType.ArbitraryFeedForward,
        calculatedFeedforward);
  }

  public void zeroEncoder(){
    // double absolutePositionInTicks = getAbsolutePositionRotations();
    // armMotor.setSelectedSensorPosition(absolutePositionInTicks);
    armMotor.setSelectedSensorPosition(offsetInTicks);
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

  public double getVelocityInRadians(){
    return Units.degreesToRadians(getVelocityInDegrees());
  }
  /**
   * Gets the arm constraints.
   *
   * @return the constraints (velocity and acceleration) for the trapezoidal
   *         profiling
   */
  public TrapezoidProfile.Constraints getConstraints() {
    return constraints;
  }

  /**
   * Sets position offset for the absolute encoder (i.e. after the offset, the
   * absolute position
   * reading will be 0 when the arm is parallel to the ground)
   */
  // public void configureEncoder() {
  //   absoluteEncoder.setPositionOffset(ArmConstants.armOffset);
  // }

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
  public double getPositionInRotations() {
    double rawPosition = armMotor.getSelectedSensorPosition()/ 114.55;
    return (rawPosition%2048)/2048;
  }

  public double getVelocityInRotations() {
    double rawVelocity = armMotor.getSelectedSensorVelocity()/ 114.55;
    return (rawVelocity%2048)/2048;
  }

  public double getPositionInDegrees() {
    return Units.rotationsToDegrees(getPositionInRotations());
  }

  public double getVelocityInDegrees() {
    return Units.rotationsToDegrees(getVelocityInRotations());
  }


  // (raw/114,55 % 2048)/2048 = x
  // raw/114.55 % 2048 = 2048*x

  public double getAbsolutePositionRotations(){
    double nativePosition = absoluteEncoder.getAbsolutePosition()*ArmConstants.armEncoderGearing;
    return nativePosition - ArmConstants.armOffset;
  }

  public double getAbsolutePositionInDegrees(){
    double absolutePositionRotations = getAbsolutePositionRotations();
    return Units.rotationsToDegrees(absolutePositionRotations);
  }


  /**
   * Gets the arm rotation in degrees.
   *
   * @return the position in degrees.
   */
  public double getAngle() {
    return getPositionInDegrees();
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
    SmartDashboard.putNumber("Arm Relative Position", armMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("ANGLE", getAngle());
    SmartDashboard.putNumber("Arm Relative Position in Rotations", getPositionInRotations());
    SmartDashboard.putNumber("Arm Relative Position in Degrees", getPositionInDegrees());
  }
}
