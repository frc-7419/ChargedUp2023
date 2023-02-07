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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.GyroSubsystem;
public class DriveBaseSubsystem extends SubsystemBase {
  private TalonFX leftLeader;
  private TalonFX leftFollower;
  private TalonFX rightLeader;
  private TalonFX rightFollower;

  final DrivetrainPoseEstimator poseEstimation;

  private Field2d field = new Field2d();
  DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.RobotConstants.kTrackWidth);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);
  PIDController leftPIDController = new PIDController(8.5, 0, 0);
  PIDController rightPIDController = new PIDController(8.5, 0, 0);

  private double leftDistance = 0;
  private double rightDistance = 0;
  private double previousTimeStamp = 0;
  private double currentTimeStamp;

  /**
   * @param gyroSubsystem
   */
  public DriveBaseSubsystem(GyroSubsystem gyroSubsystem) {
    SmartDashboard.putData("Field", field);
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

    poseEstimation = new DrivetrainPoseEstimator(gyroSubsystem);
  }

  public enum TurnDirection {
    LEFT,
    RIGHT,
  }
  /**
   * @return
   */
  // accessors
  public TalonFX getLeftMast() {
    return leftLeader;
  }
  /**
   * @return
   */
  public TalonFX getRightMast() {
    return rightLeader;
  }
  /**
   * @return
   */
  public TalonFX getLeftFollow() {
    return leftFollower;
  }
  /**
   * @return
   */
  public TalonFX getRightFollow() {
    return rightFollower;
  }
  /**
   * @param voltage
   */
  public void setLeftVoltage(double voltage) {
    leftLeader.set(ControlMode.PercentOutput, voltage / 11);
    leftFollower.set(ControlMode.PercentOutput, voltage / 11);
  }
  /**
   * @param voltage
   */
  public void setRightVoltage(double voltage) {
    rightLeader.set(ControlMode.PercentOutput, voltage / 11);
    rightFollower.set(ControlMode.PercentOutput, voltage / 11);
  }
  /**
   * @param voltage
   */
  public void setAllVoltage(double voltage) {
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }
  /**
   * @param power
   */
  public void setLeftPower(double power) {
    leftLeader.set(ControlMode.PercentOutput, power);
    leftFollower.set(ControlMode.PercentOutput, power);
  }
  /**
   * @param power
   */
  public void setRightPower(double power) {
    rightLeader.set(ControlMode.PercentOutput, power);
    rightFollower.set(ControlMode.PercentOutput, power);
  }
  /**
   * @param power
   */
  public void setAllPower(double power) {
    setLeftPower(power);
    setRightPower(power);
  }

  public void stop() {
    setAllPower(0);
  }

  /**
   * @param mode
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

  public double getLeftVelocity() {
    return leftLeader.getSelectedSensorVelocity(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocityInMeters(), getRightVelocityInMeters());
  }

  public double getRightVelocity() {
    return rightLeader.getSelectedSensorVelocity(0);
  }

  public double getLeftVelocityInMeters() {
    return getLeftVelocity()
        * Constants.RobotConstants.kWheelCircumference
        / Constants.RobotConstants.TalonFXTicksPerRotation;
  }

  public double getRightVelocityInMeters() {
    return getRightVelocity()
        * Constants.RobotConstants.kWheelCircumference
        / Constants.RobotConstants.TalonFXTicksPerRotation;
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
   * @param pose
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

  public double getDist() {
    return poseEstimation.getVisionInformation()[0];
  }

  public double getAngle() {
    return poseEstimation.getVisionInformation()[1];
  }

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

    field.setRobotPose(getCtrlsPoseEstimate());
    SmartDashboard.putNumber("Dist to Target", getDist());
    SmartDashboard.putNumber("Angle to Target", getAngle());
    poseEstimation.update(this.leftDistance, this.rightDistance);
    previousTimeStamp = currentTimeStamp;
  }

  /**
   * @param .
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    setLeftVoltage(leftVolts);
    setRightVoltage(rightVolts);
  }
}
