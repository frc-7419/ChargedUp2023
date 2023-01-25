// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.GyroSubsystem;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Performs estimation of the drivetrain's current position on the field, using
 * a vision system,
 * drivetrain encoders, and a gyroscope. These sensor readings are fused
 * together using a Kalman
 * filter. This in turn creates a best-guess at a Pose2d of where our drivetrain
 * is currently at.
 */
public class DrivetrainPoseEstimator extends SubsystemBase {
    // Sensors used as part of the Pose Estimation
    private GyroSubsystem gyroSubsystem;
    private PhotonCamera cam;
    private DriveBaseSubsystem driveBaseSubsystem;
    private PhotonPipelineResult result;
    private double resultTimeStamp;
    private double previousTimeStamp;
    private int fiducialId;

    private Map<Integer, Pose3d> poses = new HashMap<Integer, Pose3d>();
    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your
    // various sensors. Smaller numbers will cause the filter to "trust" the
    // estimate from that particular
    // component more than the others. This in turn means the particualr component
    // will have a stronger
    // influence on the final pose estimate.
    Matrix<N5, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05);
    Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(0.02, 0.01, Units.degreesToRadians(1));
    Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.02, 0.01, Units.degreesToRadians(1));

    private final DifferentialDrivePoseEstimator m_poseEstimator;

    public DrivetrainPoseEstimator(GyroSubsystem gyroSubsystem) {
        this.gyroSubsystem = gyroSubsystem;
        cam = new PhotonCamera("terima");
        poses.put(1, new Pose3d(15.513558, 1.071626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(0))));
        poses.put(2, new Pose3d(15.513558, 2.748026, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(0))));
        poses.put(3, new Pose3d(15.513558, 3.738626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(0))));
        // poses.put(4, new Pose3d(16.178784, 6.749796, new Rotation3d(0, 0.695452, Units.degreesToRadians(0))));
        // poses.put(5, new Pose3d(0.36195, 6.749796, new Rotation3d(0, 0.695452, Units.degreesToRadians(180))));
        poses.put(6, new Pose3d(1.02743, 3.738626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(180))));
        poses.put(7, new Pose3d(1.02743, 2.748026, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(180))));
        poses.put(8, new Pose3d(1.02743, 1.071626,0.462788,  new Rotation3d(0, 0, Units.degreesToRadians(180))));
        m_poseEstimator = new DifferentialDrivePoseEstimator(
                Constants.kDtKinematics,
                getRotation2d(),
                0, // Assume zero encoder counts at start
                0,
                new Pose2d(),
                localMeasurementStdDevs,
                visionMeasurementStdDevs);
    }

    /**
     * Perform all periodic pose estimation tasks.
     *
     * @param actWheelSpeeds Current Speeds (in m/s) of the drivetrain wheels
     * @param leftDist       Distance (in m) the left wheel has traveled
     * @param rightDist      Distance (in m) the right wheel has traveled
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyroSubsystem.getYaw());
    }
    public void update(double leftDist, double rightDist) {
        m_poseEstimator.update(getRotation2d(), leftDist, rightDist);
        result = cam.getLatestResult();
        resultTimeStamp = result.getTimestampSeconds();
        if (result.hasTargets() && resultTimeStamp != previousTimeStamp) {
            previousTimeStamp = resultTimeStamp;
            PhotonTrackedTarget target = result.getBestTarget();
            int fiducialId = target.getFiducialId();
            if (target.getPoseAmbiguity() <= .2) {
                Pose3d targetPose = poses.get(fiducialId);
                Transform3d camToTargetTrans = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTargetTrans.inverse()); // this lines uses where the target
                                                                                     // is on the field physically and
                                                                                     // gets the camera pose
                m_poseEstimator.addVisionMeasurement(
                        camPose.transformBy(Constants.kCameraToRobot).toPose2d(), resultTimeStamp);
                SmartDashboard.putNumber("Vision+Odo X Pos", getPoseEst().getX());
                SmartDashboard.putNumber("Vision+Odo Y Pos", getPoseEst().getY());
                SmartDashboard.putNumber("Vision+Odo Theta", getPoseEst().getRotation().getDegrees());
            }
        }
    }

    public double[] getInfo() {
        PhotonPipelineResult result = cam.getLatestResult();
        double[] info = new double[2];
        if (result.hasTargets()) {
            info[0] = PhotonUtils.calculateDistanceToTargetMeters(Constants.VisionConstants.kCameraHeight,
                    Constants.VisionConstants.kTargetHeight, Units.degreesToRadians(Constants.VisionConstants.camPitch),
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
            info[1] = result.getBestTarget().getYaw();
        }
        return info;
    }

    @Override
    public void periodic() {
    }

    /**
     * Force the pose estimator to a particular pose. This is useful for indicating
     * to the software
     * when you have manually moved your robot in a particular position on the field
     * (EX: when you
     * place it on the field at the start of the match).
     */
    public void resetToPose(Pose2d pose, double leftDist, double rightDist) {
        m_poseEstimator.resetPosition(getRotation2d(), leftDist, rightDist, pose);
    }

    /** @return The current best-guess at drivetrain position on the field. */
    public Pose2d getPoseEst() {
        return m_poseEstimator.getEstimatedPosition();
    }
}
