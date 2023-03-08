package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public final class AprilTagPositionConstants {
  public static final Pose3d kAprilTagOnePose =
      new Pose3d(15.513558, 1.071626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(180)));

  public static final Pose3d kAprilTagTwoPose =
      new Pose3d(15.513558, 2.748026, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(180)));

  public static final Pose3d kAprilTagThreePose =
      new Pose3d(15.513558, 3.738626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(180)));

  public static final Pose3d kAprilTagFourPose =
      new Pose3d(
          16.178784, 6.749796, 0.695452, new Rotation3d(0, 0.695452, Units.degreesToRadians(180)));

  public static final Pose3d kAprilTagFivePose =
      new Pose3d(
          0.36195, 6.749796, 0.695452, new Rotation3d(0, 0.695452, Units.degreesToRadians(0)));

  public static final Pose3d kAprilTagSixPose =
      new Pose3d(1.02743, 3.738626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d kAprilTagSevenPose =
      new Pose3d(1.02743, 2.748026, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d kAprilTagEightPose =
      new Pose3d(1.02743, 1.071626, 0.462788, new Rotation3d(0, 0, Units.degreesToRadians(0)));
}
