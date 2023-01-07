package frc.robot.subsystems.drive;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Implements a controller for the drivetrain. Converts a set of chassis motion commands into motor
 * controller PWM values which attempt to speed up or slow down the wheels to match the desired
 * speed.
 */
public class DriveBaseSubsystem extends SubsystemBase {
    // PWM motor controller output definitions
    private TalonFX leftLeader = new TalonFX(Constants.kDtLeftLeaderPin);
    private TalonFX leftFollower = new TalonFX(Constants.kDtLeftFollowerPin);
    private TalonFX rightLeader = new TalonFX(Constants.kDtRightLeaderPin);
    private TalonFX rightFollower = new TalonFX(Constants.kDtRightFollowerPin);


    // Drivetrain wheel speed sensors
    // Used both for speed control and pose estimation.
    Encoder leftEncoder = new Encoder(Constants.kDtLeftEncoderPinA, Constants.kDtLeftEncoderPinB);
    Encoder rightEncoder = new Encoder(Constants.kDtRightEncoderPinA, Constants.kDtRightEncoderPinB);

    // Drivetrain Pose Estimation
    final DrivetrainPoseEstimator poseEst;

    // Kinematics - defines the physical size and shape of the drivetrain, which is
    // required to convert from
    // chassis speed commands to wheel speed commands.
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.kTrackWidth);

    // Closed-loop PIDF controllers for servoing each side of the drivetrain to a
    // specific speed.
    // Gains are for example purposes only - must be determined for your own robot!
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);
    PIDController leftPIDController = new PIDController(8.5, 0, 0);
    PIDController rightPIDController = new PIDController(8.5, 0, 0);

    public DriveBaseSubsystem() {
        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // leftEncoder.setDistancePerPulse(
        //         2 * Math.PI * Constants.kWheelRadius / Constants.kEncoderResolution);
        // rightEncoder.setDistancePerPulse(
        //         2 * Math.PI * Constants.kWheelRadius / Constants.kEncoderResolution);

        leftLeader.
        leftEncoder.reset();
        rightEncoder.reset();

        rightGroup.setInverted(true);

        poseEst = new DrivetrainPoseEstimator(leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    /**
     * Given a set of chassis (fwd/rev + rotate) speed commands, perform all periodic tasks to assign
     * new outputs to the motor controllers.
     *
     * @param xSpeed Desired chassis Forward or Reverse speed (in meters/sec). Positive is forward.
     * @param rot Desired chassis rotation speed in radians/sec. Positive is counter-clockwise.
     */
    public void drive(double xSpeed, double rot) {
        // Convert our fwd/rev and rotate commands to wheel speed commands
        DifferentialDriveWheelSpeeds speeds =
                kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot));

        // Calculate the feedback (PID) portion of our motor command, based on desired
        // wheel speed
        var leftOutput = leftPIDController.calculate(leftLeader.getSelectedSensorVelocity(0) * 2 * Math.PI * Constants.RobotConstants.kWheelRadius / Constants.RobotConstants.TalonFXTicksPerRotation, speeds.leftMetersPerSecond);
        var rightOutput =
                rightPIDController.calculate(rightLeader.getSelectedSensorVelocity(0) * 2 * Math.PI * Constants.RobotConstants.kWheelRadius / Constants.RobotConstants.TalonFXTicksPerRotation, speeds.leftMetersPerSecond);

        // Calculate the feedforward (F) portion of our motor command, based on desired
        // wheel speed
        var leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

        // Update the motor controllers with our new motor commands
        leftLeader.setVoltage(leftOutput + leftFeedforward); //TODO: convert to talon.set(controlmode stuff, ok)
        leftFollower.setVoltage(leftOutput + leftFeedforward);
        rightLeader.setVoltage(rightOutput + rightFeedforward);
        rightFollower.setVoltage(rightOutput + rightFeedforward);

        // Update the pose estimator with the most recent sensor readings.
        poseEst.update(leftLeader.getSelectedSensorPosition(0), rightLeader.getSelectedSensorPosition(0));
    }

    /**
     * Force the pose estimator and all sensors to a particular pose. This is useful for indicating to
     * the software when you have manually moved your robot in a particular position on the field (EX:
     * when you place it on the field at the start of the match).
     *
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        leftLeader.reset();
        rightLeader.reset();
        poseEst.resetToPose(pose, leftLeader.getSelectedSensorPosition(0), rightLeader.getSelectedSensorPosition(0));
    }

    /** @return The current best-guess at drivetrain Pose on the field. */
    public Pose2d getCtrlsPoseEstimate() {
        return poseEst.getPoseEst();
    }
    
  @Override
  public void periodic() {
  }

  public enum TurnDirection {
    LEFT,
    RIGHT,
  }

  // accessors
  public TalonFX getLeftMast(){return leftLeader;}
  public TalonFX getRightMast(){return rightLeader;}
  public TalonFX getLeftFollow(){return leftFollower;}
  public TalonFX getRightFollow(){return rightFollower;}

  public void setLeftVoltage(double voltage){ //comment this method out as well
    leftLeader.set(ControlMode.PercentOutput, voltage/11);
    leftFollower.set(ControlMode.PercentOutput, voltage/11);
  }

  public void setRightVoltage(double voltage){ //comment this method out as well
    rightLeader.set(ControlMode.PercentOutput, voltage/11);
    rightFollower.set(ControlMode.PercentOutput, voltage/11);
  }

  public void setAllVoltage(double voltage){ //comment this method out as well
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }

  public void setLeftPower(double power){
    leftLeader.set(ControlMode.PercentOutput, power);
    leftFollower.set(ControlMode.PercentOutput, power);
  }

  public void setRightPower(double power){
    rightLeader.set(ControlMode.PercentOutput, power);
    rightFollower.set(ControlMode.PercentOutput, power);
  }

  public void setAllPower(double power){
    setLeftPower(power);
    setRightPower(power);
  }

  public void stop(){setAllPower(0);}

  public void setAllMode(NeutralMode mode){
    rightLeader.setNeutralMode(mode);
    rightFollower.setNeutralMode(mode);
    leftLeader.setNeutralMode(mode);
    leftFollower.setNeutralMode(mode);
  }

  public void brake(){setAllMode(NeutralMode.Brake);}

  public void coast(){setAllMode(NeutralMode.Coast);}

  public double getLeftVelocity(){return leftLeader.getSelectedSensorVelocity();}
  public double getRightVelocity(){return rightLeader.getSelectedSensorVelocity();}

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
}