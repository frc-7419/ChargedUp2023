package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.constants.DriveConstants;

public class GenerateTrajectory extends RamseteCommand {
  private final DriveBaseSubsystem drivetrain;

  // ignore the unused warning, this trajectory is used
  private final Trajectory trajectory;

  public GenerateTrajectory(DriveBaseSubsystem drivetrain, Trajectory trajectory) {
    super(
        trajectory,
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
    this.trajectory = trajectory;
  }

  @Override
  public void initialize() {
    drivetrain.resetOdometry(trajectory.getInitialPose());
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    // drivetrain.resetOdometry(trajectory.getInitialPose());
    super.end(interrupted);
  }
}
