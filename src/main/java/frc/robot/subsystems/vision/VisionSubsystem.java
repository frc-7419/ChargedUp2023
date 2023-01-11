// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
  
  private PhotonCamera cam1;
  private PhotonCamera cam2;
  private List<PhotonTrackedTarget> camTargets;
  private List<PhotonTrackedTarget> cam1Targets;
  private List<PhotonTrackedTarget> cam2Targets;
  private Map<String, Double> bestVal;
  private Map<String, Double> val1;
  private Map<String, Double> val2;

  public VisionSubsystem() {
    cam1 = new PhotonCamera("7419");
    // cam2 = new PhotonCamera("teri_paapa");
  }

  @Override
  public void periodic() {
    // PhotonPipelineResult[] results = { cam1.getLatestResult(), cam2.getLatestResult() };
    // boolean[] hasTargets = {results[0].hasTargets(), results[1].hasTargets()};
    
    PhotonPipelineResult[] results = { cam1.getLatestResult()};
    boolean[] hasTargets = {results[0].hasTargets()};
    SmartDashboard.putBoolean("Cam 1 Targets", hasTargets[0]);


    // SmartDashboard.putBoolean("Cam 2 Targets", true);

    camTargets = addTargets(results);
    
    if (hasTargets[0]) {
      val1 = setValues(results[0]);
      // val2 = setValues(results[1]);
      // bestVal = (val1.get("area") > val2.get("area")) ? val1 : val2;
      bestVal = val1;
    }
  }

  public List<PhotonTrackedTarget> addTargets(PhotonPipelineResult[] results) {
    cam1Targets = results[0].getTargets();
    // cam2Targets = results[1].getTargets();
    for (PhotonTrackedTarget target : cam1Targets) {
      camTargets.add(target);
    }
    // for (PhotonTrackedTarget target : cam2Targets) {
    //   camTargets.add(target);
    // }
    return camTargets;
  }
  
  public Map<String, Double> setValues(PhotonPipelineResult results) {
    Map<String, Double> values = new HashMap<String, Double>();
    values.put("yaw", results.getBestTarget().getYaw());
    values.put("pitch", results.getBestTarget().getPitch());
    values.put("area", results.getBestTarget().getArea());
    values.put("skew", results.getBestTarget().getSkew());
    return values;
  }

  public double getYaw() {
      return bestVal.get("yaw");
  }
  public double getPitch() {
    return bestVal.get("pitch");
  }
  public double getArea() {
    return bestVal.get("area");
  }
  public double getSkew() {
    return bestVal.get("skew");
  }

}
