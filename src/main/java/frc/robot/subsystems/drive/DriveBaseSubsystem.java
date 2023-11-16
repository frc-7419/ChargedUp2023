package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;

/** Subsystem that manages the drivetrain */
public class DriveBaseSubsystem extends SubsystemBase {
  private TalonFX leftLeader;
  private TalonFX leftFollower;
  private TalonFX rightLeader;
  private TalonFX rightFollower;

  final DrivetrainPoseEstimator poseEstimation;

  private Field2d field = new Field2d();
  DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(RobotConstants.kTrackWidth);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);
  PIDController leftPIDController = new PIDController(8.5, 0, 0);
  PIDController rightPIDController = new PIDController(8.5, 0, 0);

  StatorCurrentLimitConfiguration statorCurrentLimitConfiguration =
      new StatorCurrentLimitConfiguration(true, 100, 0, 0);
  SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration =
      new SupplyCurrentLimitConfiguration(true, 35, 0, 0);

  private double leftDistance = 0;
  private double rightDistance = 0;
  private double previousTimeStamp = 0;
  private double currentTimeStamp;
  private GyroSubsystem gyroSubsystem;
  /**
   * Constructor of the robot. Includes a voltage saturation and voltage compensation command to
   * ensure consistent voltage
   *
   * @param gyroSubsystem
   */
  public DriveBaseSubsystem(GyroSubsystem gyroSubsystem) {
    SmartDashboard.putData("Field", field);
    this.gyroSubsystem = gyroSubsystem;
    leftLeader = new TalonFX(DeviceIDs.CanIds.leftFalcon1.id);
    leftFollower = new TalonFX(DeviceIDs.CanIds.leftFalcon2.id);
    rightLeader = new TalonFX(DeviceIDs.CanIds.rightFalcon1.id);
    rightFollower = new TalonFX(DeviceIDs.CanIds.rightFalcon2.id);

    leftLeader.configMotionCruiseVelocity(currentTimeStamp);

    factoryResetAll();
    setAllDefaultInversions();

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    leftLeader.configVoltageCompSaturation(11);
    leftLeader.enableVoltageCompensation(true);

    leftFollower.configVoltageCompSaturation(11);
    leftFollower.enableVoltageCompensation(true);

    rightLeader.configVoltageCompSaturation(11);
    rightLeader.enableVoltageCompensation(true);

    rightFollower.configVoltageCompSaturation(11);
    rightFollower.enableVoltageCompensation(true);

    poseEstimation = new DrivetrainPoseEstimator(gyroSubsystem);
  }
  
  public void setPIDFConstants(int slot, TalonFX motor, double kP, double kI, double kD, double kF) {
    motor.config_kP(slot, kP, 0);
    motor.config_kI(slot, kI, 0);
    motor.config_kD(slot, kD, 0);
    motor.config_kF(slot, kF, 0);
  }

  /**
   * Current limits so that our drivetrain doesn't have any electrical overloads.
   *
   * @param motorController
   */

  private void configCurrentLimits(TalonFX motorController) {
    motorController.configSupplyCurrentLimit(supplyCurrentLimitConfiguration, 5);
    motorController.configStatorCurrentLimit(statorCurrentLimitConfiguration, 5);
  }

  public enum TurnDirection {
    LEFT,
    RIGHT
  }
  // accessors
  public TalonFX getLeftMast() {
    return leftLeader;
  }

  public TalonFX getRightMast() {
    return rightLeader;
  }

  public TalonFX getLeftFollow() {
    return leftFollower;
  }

  public TalonFX getRightFollow() {
    return rightFollower;
  }
  /**
   * Provides a specific voltage to the left side of the drivetrain.
   *
   * @param voltage voltage to set
   */
  public void setLeftVoltage(double voltage) {
    leftLeader.set(ControlMode.PercentOutput, voltage / 11);
    leftFollower.set(ControlMode.PercentOutput, voltage / 11);
  }
  /**
   * Provides a specific voltage to the right side of the drivetrain.
   *
   * @param voltage voltage to set
   */
  public void setRightVoltage(double voltage) {
    rightLeader.set(ControlMode.PercentOutput, voltage / 11);
    rightFollower.set(ControlMode.PercentOutput, voltage / 11);
  }
  /**
   * Provides a specific voltage to the drivetrain.
   *
   * @param voltage voltage to set
   */
  public void setAllVoltage(double voltage) {
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }

  public void setTopPower(double power) {
    leftLeader.set(ControlMode.PercentOutput, power);
    rightLeader.set(ControlMode.PercentOutput, power);
  }

  public void setBottomPower(double power) {
    leftFollower.set(ControlMode.PercentOutput, power);
    rightFollower.set(ControlMode.PercentOutput, power);
  }
  /**
   * Sets the power of the left side of the drivetrain.
   *
   * @param power Power (-1 to 1) to set
   */
  public void setLeftPower(double power) {
    leftLeader.set(ControlMode.PercentOutput, power);
    leftFollower.set(ControlMode.PercentOutput, power);
  }
  /**
   * Sets the power of the right side of the drivetrain.
   *
   * @param power Power (-1 to 1) to set
   */
  public void setRightPower(double power) {
    rightLeader.set(ControlMode.PercentOutput, power);
    rightFollower.set(ControlMode.PercentOutput, power);
  }
  /**
   * Sets the power of the drivetrain.
   *
   * @param power Power (-1 to 1) to set
   */
  public void setAllPower(double power) {
    setLeftPower(power);
    setRightPower(power);
  }
  /** Stops the drivetrain (sets all motors to 0 power) */
  public void stop() {
    setAllPower(0);
  }
  /**
   * Sets the Neutral Mode (used when the motor is not running) of the drivetrain.
   *
   * @param mode Neutral Mode (Coast or Brake) to set
   */
  public void setAllMode(NeutralMode mode) {
    rightLeader.setNeutralMode(mode);
    rightFollower.setNeutralMode(mode);
    leftLeader.setNeutralMode(mode);
    leftFollower.setNeutralMode(mode);
  }

  public void brake() {
    setAllMode(NeutralMode.Brake);
  }

  public void coast() {
    setAllMode(NeutralMode.Coast);
  }
  /**
   * Gets the velocity of the left side of the drivetrain
   *
   * @return Velocity, in raw sensor units
   */
  public double getLeftVelocity() {
    return leftLeader.getSelectedSensorVelocity(0);
  }
  /**
   * Gets the wheels speeds of both sides of the drivetrain
   *
   * @return The speed of the wheels on the left and right side of the drivetrain, as a {@link
   *     DifferentialDriveWheelSpeeds} object
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocityInMeters(), getRightVelocityInMeters());
  }
  /**
   * Gets the velocity of the right side of the drivetrain
   *
   * @return Velocity, in raw sensor units
   */
  public double getRightVelocity() {
    return rightLeader.getSelectedSensorVelocity(0);
  }
  /**
   * Gets the velocity of the left side of the drivetrain
   *
   * @return Velocity, in meters per second
   */
  public double getLeftVelocityInMeters() {
    return getLeftVelocity()
        * RobotConstants.kWheelCircumference
        / RobotConstants.TalonFXTicksPerRotation;
  }
  /**
   * Gets the velocity of the right side of the drivetrain
   *
   * @return Velocity, in meters per second
   */
  public double getRightVelocityInMeters() {
    return getRightVelocity()
        * RobotConstants.kWheelCircumference
        / RobotConstants.TalonFXTicksPerRotation;
  }
  /** Set all the drivetrain motors to the correct inversions */
  public void setAllDefaultInversions() {
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
    leftLeader.setInverted(false);
    leftFollower.setInverted(false);
  }
  /** Set all the drivetrain motors to factory default */
  public void factoryResetAll() {
    rightLeader.configFactoryDefault();
    rightFollower.configFactoryDefault();
    leftLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();
  }
  /**
   * Arcade drives the robot using the given linear and rotational speeds.
   *
   * @param xSpeed Linear speed to drive at
   * @param rot Rotational speed to drive at
   */
  public void drive(double xSpeed, double rot) {
    // Convert our fwd/rev and rotate commands to wheel speed commands
    DifferentialDriveWheelSpeeds speeds =
        kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot));

    currentTimeStamp = Timer.getFPGATimestamp();

    double leftDistance = getLeftVelocityInMeters() * (currentTimeStamp - previousTimeStamp);
    double rightDistance = getRightVelocityInMeters() * (currentTimeStamp - previousTimeStamp);

    this.leftDistance += leftDistance;
    this.rightDistance += rightDistance;

    double leftOutput = leftPIDController.calculate(leftDistance, speeds.leftMetersPerSecond);
    double rightOutput = rightPIDController.calculate(rightDistance, speeds.rightMetersPerSecond);

    var leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    setLeftVoltage(leftOutput + leftFeedforward);
    setRightVoltage(rightOutput + rightFeedforward);
  }
  /**
   * Force the pose estimator and all sensors to a particular pose. This is useful for indicating to
   * the software when you have manually moved your robot in a particular position on the field (EX:
   * when you place it on the field at the start of the match).
   *
   * @param pose {@link Pose2d} to reset to
   */
  public void resetOdometry(Pose2d pose) {
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    poseEstimation.resetToPose(pose, 0, 0);
  }
  /**
   * @return The current best-guess at drivetrain Pose on the field.
   */
  public Pose2d getCtrlsPoseEstimate() {
    return poseEstimation.getPoseEstimation();
  }
  /**
   * Gets the distance to the nearest AprilTag target
   *
   * @return Distance to the nearest AprilTag, in meters
   */
  public double getDist() {
    return poseEstimation.getVisionInformation()[0];
  }
  /**
   * Gets the yaw angle to the nearest AprilTag target
   *
   * @return Yaw to the nearest AprilTag, in degrees
   */
  public double getAngle() {
    return poseEstimation.getVisionInformation()[1];
  }
  /** Tne periodic function is used for odometry */
  @Override
  public void periodic() {
    currentTimeStamp = Timer.getFPGATimestamp();
    double leftDistance = getLeftVelocityInMeters() * (currentTimeStamp - previousTimeStamp);
    double rightDistance = getRightVelocityInMeters() * (currentTimeStamp - previousTimeStamp);
    this.leftDistance += leftDistance;
    this.rightDistance += rightDistance;

    SmartDashboard.putNumber("Odo X Pos", getCtrlsPoseEstimate().getX());
    SmartDashboard.putNumber("Odo Y Pos", getCtrlsPoseEstimate().getY());
    SmartDashboard.putNumber("Odo Theta", getCtrlsPoseEstimate().getRotation().getDegrees());
    SmartDashboard.putNumber("left velocity", getLeftVelocityInMeters());

    field.setRobotPose(getCtrlsPoseEstimate());
    SmartDashboard.putNumberArray("Odometry", new double[]{getCtrlsPoseEstimate().getX(), getCtrlsPoseEstimate().getY(), gyroSubsystem.getYaw()});
    SmartDashboard.putNumber("Dist to Target", getDist());
    SmartDashboard.putNumber("Angle to Target", getAngle());
    poseEstimation.update(this.leftDistance, this.rightDistance);
    previousTimeStamp = currentTimeStamp;
  }

  /**
   * Tank drives the robot using the provided left and right voltages
   *
   * @param leftVolts Voltage to supply to the left side of the drivetrain
   * @param rightVolts Voltage to supply to the right side of the drivetrain
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    setLeftVoltage(leftVolts);
    setRightVoltage(rightVolts);
  }
}
