// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.Optional;
import frc.robot.Constants;

public class PoseEstimatorVisionSubsystem extends SubsystemBase {
  private PhotonCamera cam1;
  private PhotonCamera cam2;
  private List<PhotonTrackedTarget> camTargets;
  private List<PhotonTrackedTarget> cam1Targets;
  private List<PhotonTrackedTarget> cam2Targets;
  private Map<String, Double> bestVal;
  private Map<String, Double> val1;
  private Map<String, Double> val2;
  private Optional<Pair<Pose3d, Double>> result;
  private RobotPoseEstimator robotPoseEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout; 
  private double camDiagFOV = 170.0;
  private double camPitch = Units.radiansToDegrees(Constants.VisionConstants.camPitch);
  private double camHeightOffGround = Constants.VisionConstants.kCameraHeight;
  private double maxLEDRange = Constants.VisionConstants.maxLEDRange;
  private int camResolutionWidth = Constants.VisionConstants.camResWidth;
  private int camResolutionHeight = Constants.VisionConstants.camResHeight;
  private double minTargetArea = Constants.VisionConstants.minTargetArea;
  private SimVisionSystem simVisionSystem;

  
  public PoseEstimatorVisionSubsystem() {
    simVisionSystem = new SimVisionSystem(Constants.VisionConstants.name1, camDiagFOV, new Transform3d(), camHeightOffGround, camResolutionWidth, camResolutionHeight, minTargetArea);

    cam1 = new PhotonCamera(Constants.VisionConstants.name1);
    Transform3d robotToCam1 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    cam2 = new PhotonCamera(Constants.VisionConstants.name1);
    Transform3d robotToCam2 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    List<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(cam1, robotToCam1));
    camList.add(new Pair<PhotonCamera, Transform3d>(cam2, robotToCam2));
    // aprilTagFieldLayout = new AprilTagFieldLayout((Path) AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile)); //update with chargedup field when merged to photonvision + wpilib and avaialble
    robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }

  @Override
  public void periodic() {
    result = robotPoseEstimator.update();
  }
}
