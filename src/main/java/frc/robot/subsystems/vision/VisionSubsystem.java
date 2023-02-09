// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera cam1;
  private Map<String, Double> bestVal;
  private Map<String, Double> val1;
  public VisionSubsystem() {
    cam1 = new PhotonCamera("terima");
    // cam2 = new PhotonCamera("teri_paapa");
  }

  @Override
  public void periodic() {
    //TODO use when the second camera arrives
    // PhotonPipelineResult[] results = { cam1.getLatestResult(),
    // cam2.getLatestResult() };
    // boolean[] hasTargets = {results[0].hasTargets(), results[1].hasTargets()};

    PhotonPipelineResult[] results = {cam1.getLatestResult()};
    boolean[] hasTargets = {results[0].hasTargets()};

    //TODO use when the second camera arrives
    SmartDashboard.putBoolean("Cam 1 Targets", hasTargets[0]);
    results[0].getTargets();

    if (hasTargets[0]) {
      val1 = setValues(results[0]);

      // val2 = setValues(results[1]);
      // bestVal = (val1.get("area") > val2.get("area")) ? val1 : val2;
      bestVal = val1;
      SmartDashboard.putNumber("Yaw", getYaw());
      SmartDashboard.putNumber("Skew", getSkew());
      SmartDashboard.putNumber("Area", getArea());
      SmartDashboard.putNumber("Pitch", getPitch());
    }
  }
  //TODO fix when second camera arrives
  // public List<PhotonTrackedTarget> addTargets(PhotonPipelineResult[] results) {
  // camTargets = new List<PhotonTrackedTarget>();
  // // cam2Targets = results[1].getTargets();
  // for (PhotonTrackedTarget target : cam1Targets) {
  // camTargets.add(target);
  // }
  // // for (PhotonTrackedTarget target : cam2Targets) {
  // // camTargets.add(target);
  // // }
  // return camTargets;
  // }

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
