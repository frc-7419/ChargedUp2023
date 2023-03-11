/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private static Alliance allianceColor;
  private static String allianceSide;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
    PathPlannerServer.startServer(5811);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    robotContainer.getAutonomousCommand().schedule();
    allianceColor = DriverStation.getAlliance();
    Map<Integer, String> locationMap = new HashMap<Integer, String>();
    locationMap.put(1, "left");
    locationMap.put(2, "mid");
    locationMap.put(3, "right");
    allianceSide = locationMap.get(DriverStation.getLocation()); // 1 - left 2 - mid 3 - right
  }

  public static String getAllianceColor() {
    if (allianceColor == DriverStation.Alliance.Blue) {
      return "Blue";
    }
    return "Red";
  }

  public static Alliance getAlliance() {
    return allianceColor;
  }

  public static String getAllianceSide(){
    return allianceSide;
  }
  

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    robotContainer.setDefaultCommands();
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
