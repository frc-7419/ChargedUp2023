// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;
import java.util.HashMap;
import java.util.Map;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Performs estimation of the drivetrain's current position on the field, using a vision system,
 * drivetrain encoders, and a gyroscope. These sensor readings are fused together using a Kalman
 * filter. This in turn creates a best-guess at a Pose2d of where our drivetrain is currently at.
 */
public class DrivetrainPoseEstimator {
  // Sensors used as part of the Pose Estimation
  private GyroSubsystem gyroSubsystem;
  private PhotonCamera cam;
  private PhotonPipelineResult result;
  private double resultTimeStamp;
  private double previousTimeStamp;

  private Map<Integer, Pose3d> poses = new HashMap<Integer, Pose3d>();
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your
  // various sensors. Smaller numbers will cause the filter to "trust" the
  // estimate from that particular
  // component more than the others. This in turn means the particualr component
  // will have a stronger
  // influence on the final pose estimate.
  Matrix<N5, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05);
  Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.01));
  Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(5));

  private final DifferentialDrivePoseEstimator m_poseEstimator;
  /**
   * Constructs a new DrivetrainPoseEstimator. Initializes the AprilTag poses and their
   * corresponding Fiducial IDs.
   *
   * @param gyroSubsystem
   */
  public DrivetrainPoseEstimator(GyroSubsystem gyroSubsystem) {
    this.gyroSubsystem = gyroSubsystem;
    cam = new PhotonCamera("terima");

    /*
    ________                    __        __       __           
    |        \                  |  \      |  \     /  \          
    \$$$$$$$$______    ______   \$$      | $$\   /  $$  ______  
      | $$  /      \  /      \ |  \      | $$$\ /  $$$ |      \ 
      | $$ |  $$$$$$\|  $$$$$$\| $$      | $$$$\  $$$$  \$$$$$$\
      | $$ | $$    $$| $$   \$$| $$      | $$\$$ $$ $$ /      $$
      | $$ | $$$$$$$$| $$      | $$      | $$ \$$$| $$|  $$$$$$$
      | $$  \$$     \| $$      | $$      | $$  \$ | $$ \$$    $$
        \$$   \$$$$$$$ \$$       \$$       \$$      \$$  \$$$$$$$                                                   
     */

    poses.put(1, Constants.AprilTagPositionConstants.kAprilTagOnePose);
    poses.put(2, Constants.AprilTagPositionConstants.kAprilTagTwoPose);
    poses.put(3, Constants.AprilTagPositionConstants.kAprilTagThreePose);
    poses.put(4, Constants.AprilTagPositionConstants.kAprilTagFourPose);
    poses.put(5, Constants.AprilTagPositionConstants.kAprilTagFivePose);
    poses.put(6, Constants.AprilTagPositionConstants.kAprilTagSixPose);
    poses.put(7, Constants.AprilTagPositionConstants.kAprilTagSevenPose);
    poses.put(8, Constants.AprilTagPositionConstants.kAprilTagEightPose);
    

    m_poseEstimator =
        new DifferentialDrivePoseEstimator(
            Constants.kDtKinematics,
            getRotation2d(),
            0, // Assume zero encoder counts at start
            0,
            new Pose2d(),
            localMeasurementStdDevs,
            visionMeasurementStdDevs);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(gyroSubsystem.getYaw());
  }

  /**
   * Perform all periodic pose estimation tasks.
   *
   * @param leftDist Distance (in m) the left wheel has traveled
   * @param rightDist Distance (in m) the right wheel has traveled
   */
  public void update(double leftDist, double rightDist) {
    m_poseEstimator.update(getRotation2d(), leftDist, rightDist);
    result = cam.getLatestResult();
    resultTimeStamp = result.getTimestampSeconds();

    if (result.hasTargets() && resultTimeStamp != previousTimeStamp) {
      previousTimeStamp = resultTimeStamp;
      PhotonTrackedTarget target = result.getBestTarget();
      int fiducialId = target.getFiducialId();

      if (target.getPoseAmbiguity() <= VisionConstants.visionAmbiguityThreshold) {
        Pose3d targetPose = poses.get(fiducialId);
        Transform3d camToTargetTrans = target.getBestCameraToTarget();
        Pose3d camPose =
            targetPose.transformBy(camToTargetTrans.inverse()); // this lines uses where the target
        // is on the field physically and
        // gets the camera pose
        m_poseEstimator.addVisionMeasurement(
            camPose.transformBy(Constants.kCameraToRobot).toPose2d(), resultTimeStamp);

        // outputting everthing to smartdashboard for viewing
        SmartDashboard.putNumber("Vision+Odo X Pos", getPoseEstimation().getX());
        SmartDashboard.putNumber("Vision+Odo Y Pos", getPoseEstimation().getY());
        SmartDashboard.putNumber(
            "Vision+Odo Theta", getPoseEstimation().getRotation().getDegrees());
      }
    }
  }
  /**
   * Return the vision information (distance and yaw) of the target
   *
   * @return
   */
  public double[] getVisionInformation() {
    PhotonPipelineResult result = cam.getLatestResult();
    double[] visionInformation = new double[2];

    if (result.hasTargets()) {
      visionInformation[0] =
          PhotonUtils.calculateDistanceToTargetMeters(
              Constants.VisionConstants.kCameraHeight,
              Constants.VisionConstants.kTargetHeight,
              Units.degreesToRadians(Constants.VisionConstants.kCameraPitch),
              Units.degreesToRadians(result.getBestTarget().getPitch()));
      visionInformation[1] = result.getBestTarget().getYaw();
    }
    return visionInformation;
  }

  /**
   * Force the pose estimator to a particular pose. This is useful for indicating to the software
   * when you have manually moved your robot in a particular position on the field (EX: when you
   * place it on the field at the start of the match).
   */
  /**
   * Reset the pose given a specified Rotation2d, left distance, right distance, and pose
   *
   * @param pose
   * @param leftDist
   * @param rightDist
   */
  public void resetToPose(Pose2d pose, double leftDist, double rightDist) {
    m_poseEstimator.resetPosition(getRotation2d(), leftDist, rightDist, pose);
  }

  /**
   * @return The current best-guess at drivetrain position on the field.
   */
  public Pose2d getPoseEstimation() {
    return m_poseEstimator.getEstimatedPosition();
  }
}
