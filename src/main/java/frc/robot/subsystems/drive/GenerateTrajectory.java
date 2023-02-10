package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;

public class GenerateTrajectory extends RamseteCommand {
  private final DriveBaseSubsystem driveBaseSubsystem;

  // ignore the unused warning, this trajectory is used
  private final Trajectory trajectory;

  /**
   * Generates trajectory based on RamseteCommand class and desired trajectory path.
   * @param driveBaseSubsystem Gets DriveBaseSubsystem class for West Coast drive.
   * @param trajectory Trajectory class
   */
  public GenerateTrajectory(DriveBaseSubsystem driveBaseSubsystem, Trajectory trajectory) {
    super(
        trajectory,
        driveBaseSubsystem::getCtrlsPoseEstimate,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kDriveKinematics,
        driveBaseSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVelocity, 0, 0),
        new PIDController(DriveConstants.kPDriveVelocity, 0, 0),
        driveBaseSubsystem::tankDriveVolts,
        driveBaseSubsystem);
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.trajectory = trajectory;
  }

  /**
   * Resets odometry when called
   */
  @Override
  public void initialize() {
    driveBaseSubsystem.resetOdometry(trajectory.getInitialPose());
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.stop();
    super.end(interrupted);
  }
}