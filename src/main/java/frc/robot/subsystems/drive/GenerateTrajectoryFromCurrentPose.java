package frc.robot.subsystems.drive;

import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.constants.DriveConstants;

import com.kauailabs.navx.frc.Tracer;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.WaypointPositionConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.GenerateTrajectoryFromCurrentPose;
import frc.robot.subsystems.gyro.SmartBalance;

import java.util.ArrayList;
import java.util.List;

public class GenerateTrajectoryFromCurrentPose extends RamseteCommand {
  private final DriveBaseSubsystem drivetrain;
  List<PathPoint> waypoints;

  // ignore the unused warning, this trajectory is used

  public GenerateTrajectoryFromCurrentPose(DriveBaseSubsystem drivetrain, List<PathPoint> waypoints) {
    super(
      PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints),
      drivetrain::getCtrlsPoseEstimate,
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
      DriveConstants.driveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVelocity, DriveConstants.kIDriveVelocity, DriveConstants.kDDriveVelocity),
      new PIDController(DriveConstants.kPDriveVelocity, DriveConstants.kIDriveVelocity, DriveConstants.kDDriveVelocity),
      drivetrain::tankDriveVolts,
      drivetrain);
    this.drivetrain = drivetrain;
    this.waypoints = waypoints;
  }

  @Override
  public void initialize() {
    drivetrain.resetOdometry(PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints).getInitialPose());
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    // drivetrain.resetOdometry(trajectory.getInitialPose());
    super.end(interrupted);
  }
}
