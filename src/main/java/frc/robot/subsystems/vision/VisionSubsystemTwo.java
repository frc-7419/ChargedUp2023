// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

public class VisionSubsystemTwo extends SubsystemBase {
  // set up a new instance of NetworkTables (the api/library used to read values
  // from limelight)
  private PhotonCamera cam1;
  private PhotonCamera cam2;
  private List<PhotonTrackedTarget> camTargets;
  private List<PhotonTrackedTarget> cam1Targets;
  private List<PhotonTrackedTarget> cam2Targets;
  private Map<String, Double> bestVal;
  private Map<String, Double> val1;
  private Map<String, Double> val2;
  private Optional<Pair<Pose3d, Double>> result;

  // AprilTagFieldLayout aprilTagFieldLayout = new ApriltagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));
  
  public VisionSubsystemTwo() {
    cam1 = new PhotonCamera("teri_ma");
      Transform3d robotToCam1 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    cam2 = new PhotonCamera("teri_paapa");
          Transform3d robotToCam2 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(cam1, robotToCam1));
    camList.add(new Pair<PhotonCamera, Transform3d>(cam2, robotToCam2));
    RobotPoseEstimator robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }

  @Override
  public void periodic() {
    result = robotPoseEstimator.update();
    // PhotonPipelineResult[] results = { cam1.getLatestResult(), cam2.getLatestResult() };
    // boolean[] hasTargets = new boolean[results.length];
    // for (int i = 0; i < results.length; i+a+) {
    //   hasTargets[i] = results[i].hasTargets();
    // }
    // SmartDashboard.putBoolean("Cam 1 Targets", hasTargets[0]);
    // SmartDashboard.putBoolean("Cam 2 Targets", hasTargets[1]);

    // camTargets = addTargets(results);
    
    // if (hasTargets[0] || hasTargets[1]) {
    //   val1 = setValues(results[0]);
    //   val2 = setValues(results[1]);
    //   bestVal = (val1.get("area") > val2.get("area")) ? val1 : val2;
    // }
  }

  // public List<PhotonTrackedTarget> addTargets(PhotonPipelineResult[] results) {
  //   cam1Targets = results[0].getTargets();
  //   cam2Targets = results[1].getTargets();
  //   for (PhotonTrackedTarget target : camTargets) {
  //     camTargets.add(target);
      
  //   }
  //   return camTargets;
  // }
  
  // public Map<String, Double> setValues(PhotonPipelineResult results) {
  //   Map<String, Double> values = new HashMap<String, Double>();
  //   values.put("yaw", results.getBestTarget().getYaw());
  //   values.put("pitch", results.getBestTarget().getPitch());
  //   values.put("area", results.getBestTarget().getArea());
  //   values.put("skew", results.getBestTarget().getSkew());
  //   return values;
  // }

  // public double getYaw() {
  //     return bestVal.get("yaw");
  // }
  // public double getPitch() {
  //   return bestVal.get("pitch");
  // }
  // public double getArea() {
  //   return bestVal.get("area");
  // }
  // public double getSkew() {
  //   return bestVal.get("skew");
  // }

}
