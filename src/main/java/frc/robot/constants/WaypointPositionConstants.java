package frc.robot.constants;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class WaypointPositionConstants {
  public static final Translation2d kStartPose = new Translation2d(0, 0);
  // single substation and portal initial location constants

  // red mid waypoints

  public static final double kRedMidFirstWayPointX = 10.67;

  public static final double kRedMidFirstWayPointY = 2.15;

  public static final double kRedMidSecondWayPointX = 11.51;

  public static final double kRedMidSecondWayPointY = 0.86;

  public static final double kRedMidThirdWayPointX = 12.99;

  public static final double kRedMidThirdWayPointY = 0.57;

  // red single substation waypoints

  public static final double kRedSubstationFirstWayPointX = 7.70;

  public static final double kRedSubstationFirstWayPointY = 6.60;

  public static final double kRedSubstationSecondWayPointX = 5.66;

  public static final double kRedSubstationSecondWayPointY = 6.60;

  // blue mid waypoints

  public static final double kBlueMidFirstWayPointX = 6.07;

  public static final double kBlueMidFirstWayPointY = 3.31;

  public static final double kBlueMidSecondWayPointX = 5.49;

  public static final double kBlueMidSecondWayPointY = 4.48;

  public static final double kBlueMidThirdWayPointX = 3.91;

  public static final double kBlueMidThirdWayPointY = 4.76;

  // blue single substation waypoints

  public static final double kBlueSubstationFirstWayPointX = 8.38;

  public static final double kBlueSubstationFirstWayPointY = 6.41;

  public static final double kBlueSubstationSecondWayPointX = 10.34;

  public static final double kBlueSubstationSecondWayPointY = 6.72;

  // waypoint angles

  public static final double kHeadingFront = 0; // degrees

  public static final double kHeadingPerpendicular = 90; // degrees

  public static final double kHeadingPerpendicularReverse = -90; // degrees

  public static final double kHeadingAvoidChargeStation = 135; // degrees

  public static final double kHeadingAvoidChargeStationReverse = -45; // degrees

  public static final double kHeadingReverse = 180; // degrees

  // constructing waypoints

  // red waypoints

  // red mid waypoints

  public static final PathPoint kRedMidFirstWayPoint =
      new PathPoint(
          new Translation2d(kRedMidFirstWayPointX, kRedMidFirstWayPointY),
          Rotation2d.fromDegrees(kHeadingFront));

  public static final PathPoint kRedMidSecondWayPoint =
      new PathPoint(
          new Translation2d(kRedMidSecondWayPointX, kRedMidSecondWayPointY),
          Rotation2d.fromDegrees(kHeadingPerpendicularReverse));

  public static final PathPoint kRedMidThirdWayPoint =
      new PathPoint(
          new Translation2d(kRedMidThirdWayPointX, kRedMidThirdWayPointY),
          Rotation2d.fromDegrees(kHeadingFront));

  // red single substation waypoints

  public static final PathPoint kRedSubstationFirstWayPoint =
      new PathPoint(
          new Translation2d(kRedSubstationFirstWayPointX, kRedSubstationFirstWayPointY),
          Rotation2d.fromDegrees(kHeadingReverse));

  public static final PathPoint kRedSubstationSecondWayPoint =
      new PathPoint(
          new Translation2d(kRedSubstationSecondWayPointX, kRedSubstationSecondWayPointY),
          Rotation2d.fromDegrees(kHeadingReverse));

  // blue waypoints

  // blue mid waypoints

  public static final PathPoint kBlueMidFirstWayPoint =
      new PathPoint(
          new Translation2d(kBlueMidFirstWayPointX, kBlueMidFirstWayPointY),
          Rotation2d.fromDegrees(kHeadingPerpendicular));

  public static final PathPoint kBlueMidSecondWayPoint =
      new PathPoint(
          new Translation2d(kBlueMidSecondWayPointX, kBlueMidSecondWayPointY),
          Rotation2d.fromDegrees(kHeadingAvoidChargeStation));

  public static final PathPoint kBlueMidThirdWayPoint =
      new PathPoint(
          new Translation2d(kBlueMidThirdWayPointX, kBlueMidThirdWayPointY),
          Rotation2d.fromDegrees(kHeadingReverse));

  // blue single substation waypoints

  public static final PathPoint kBlueSubstationFirstWayPoint =
      new PathPoint(
          new Translation2d(kBlueSubstationFirstWayPointX, kBlueSubstationFirstWayPointY),
          Rotation2d.fromDegrees(kHeadingFront));

  public static final PathPoint kBlueSubstationSecondWayPoint =
      new PathPoint(
          new Translation2d(kBlueSubstationSecondWayPointX, kBlueSubstationSecondWayPointY),
          Rotation2d.fromDegrees(kHeadingFront));
}
