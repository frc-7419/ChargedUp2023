package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.GyroSubsystem;

public class DriveBaseSubsystem extends SubsystemBase {
  private TalonFX leftLeader;
  private TalonFX leftFollower;
  private TalonFX rightLeader;
  private TalonFX rightFollower;
  
  final DrivetrainPoseEstimator poseEst;
  
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.RobotConstants.kTrackWidth);
  
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);
  PIDController leftPIDController = new PIDController(8.5, 0, 0);
  PIDController rightPIDController = new PIDController(8.5, 0, 0);
  
  private double ld = 0;
  private double rd = 0;
  private double previousTimeStamp = 0;
  private double currentTimeStamp;

  public DriveBaseSubsystem(GyroSubsystem gyroSubsystem) {
    leftLeader = new TalonFX(Constants.CanIds.leftFalcon1.id);
    leftFollower = new TalonFX(Constants.CanIds.leftFalcon2.id);
    rightLeader = new TalonFX(Constants.CanIds.rightFalcon1.id);
    rightFollower = new TalonFX(Constants.CanIds.rightFalcon2.id);
    factoryResetAll();
    setAllDefaultInversions();

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.configVoltageCompSaturation(11);
    leftLeader.enableVoltageCompensation(true);

    leftFollower.configVoltageCompSaturation(11);
    leftFollower.enableVoltageCompensation(true);

    rightLeader.configVoltageCompSaturation(11);
    rightFollower.enableVoltageCompensation(true);

    rightFollower.configVoltageCompSaturation(11);
    rightFollower.enableVoltageCompensation(true);

    poseEst = new DrivetrainPoseEstimator(gyroSubsystem);
  }

  public enum TurnDirection {
    LEFT,
    RIGHT,
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

  public void setLeftVoltage(double voltage) {
    leftLeader.set(ControlMode.PercentOutput, voltage / 11);
    leftFollower.set(ControlMode.PercentOutput, voltage / 11);
  }

  public void setRightVoltage(double voltage) {
    rightLeader.set(ControlMode.PercentOutput, voltage / 11);
    rightFollower.set(ControlMode.PercentOutput, voltage / 11);
  }

  public void setAllVoltage(double voltage) {
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }

  public void setLeftPower(double power) {
    leftLeader.set(ControlMode.PercentOutput, power);
    leftFollower.set(ControlMode.PercentOutput, power);
  }

  public void setRightPower(double power) {
    rightLeader.set(ControlMode.PercentOutput, power);
    rightFollower.set(ControlMode.PercentOutput, power);
  }

  public void setAllPower(double power) {
    setLeftPower(power);
    setRightPower(power);
  }

  public void stop() {
    setAllPower(0);
  }

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

  public double getLeftVelocity() {
    return leftLeader.getSelectedSensorVelocity(0);
  }

  public double getRightVelocity() {
    return rightLeader.getSelectedSensorVelocity(0);
  }

  public double getLeftVelocityInMeters() {
    return getLeftVelocity()*Constants.RobotConstants.kWheelCircumference/Constants.RobotConstants.TalonFXTicksPerRotation;
  }

  public double getRightVelocityInMeters() {
    return getRightVelocity()*Constants.RobotConstants.kWheelCircumference/Constants.RobotConstants.TalonFXTicksPerRotation;
  }

  public void setAllDefaultInversions() {
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
    leftLeader.setInverted(false);
    leftFollower.setInverted(false);
  }

  public void factoryResetAll() {
    rightLeader.configFactoryDefault();
    rightFollower.configFactoryDefault();
    leftLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();
  }

  public void drive(double xSpeed, double rot) {
    // Convert our fwd/rev and rotate commands to wheel speed commands
    DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot));

    // Calculate the feedback (PID) portion of our motor command, based on desired
    // wheel speed

    currentTimeStamp = Timer.getFPGATimestamp();

    double leftDistance = getLeftVelocityInMeters() * (currentTimeStamp - previousTimeStamp);
    double rightDistance = getRightVelocityInMeters() * (currentTimeStamp - previousTimeStamp);

    ld+=leftDistance;
    rd+=rightDistance;

    double leftOutput = leftPIDController.calculate(leftDistance,
        speeds.leftMetersPerSecond);
    double rightOutput = rightPIDController.calculate(rightDistance,
        speeds.rightMetersPerSecond);

    // Calculate the feedforward (F) portion of our motor command, based on desired
    // wheel speed
    var leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    // Update the motor controllers with our new motor commands
    setLeftVoltage(leftOutput + leftFeedforward);
    setRightVoltage(rightOutput + rightFeedforward);
    // Update the pose estimator with the most recent sensor readings.
    // poseEst.update(leftDistance, rightDistance);
    // poseEst.update(ld, rd);
  }

  /**
   * Force the pose estimator and all sensors to a particular pose. This is useful
   * for indicating to
   * the software when you have manually moved your robot in a particular position
   * on the field (EX:
   * when you place it on the field at the start of the match).
   *
   * @param pose
   */
  public double getDist() {
    return poseEst.getInfo()[0];
  }
  public double getAngle() {
    return poseEst.getInfo()[1];
  }
  public void resetOdometry(Pose2d pose) {
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    poseEst.resetToPose(pose, 0, 0);
  }

  /** @return The current best-guess at drivetrain Pose on the field. */
  public Pose2d getCtrlsPoseEstimate() {
    return poseEst.getPoseEst();
  }
  @Override
  public void periodic() {
    currentTimeStamp = Timer.getFPGATimestamp();
    double leftDistance = getLeftVelocityInMeters() * (currentTimeStamp - previousTimeStamp);
    double rightDistance = getRightVelocityInMeters() * (currentTimeStamp - previousTimeStamp);
    ld+=leftDistance;
    rd+=rightDistance;
//     SmartDashboard.putNumber("cumulative left distance", ld);
//     SmartDashboard.putNumber("cumulative right distance", rd);
//     SmartDashboard.putNumber("time step left distance", leftDistance);
//     SmartDashboard.putNumber("time step right distance", rightDistance);

    SmartDashboard.putNumber("Odo X Pos", getCtrlsPoseEstimate().getX());
    SmartDashboard.putNumber("Odo Y Pos", getCtrlsPoseEstimate().getY());
    SmartDashboard.putNumber("Odo Theta", getCtrlsPoseEstimate().getRotation().getDegrees());

    SmartDashboard.putNumber("Dist to Target", getDist());
    SmartDashboard.putNumber("Angle to Target", getAngle());
    poseEst.update(ld, rd);
    previousTimeStamp = currentTimeStamp;
  }

}
