package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;

public class GenerateTrajectory extends RamseteCommand {
  private final DriveBaseSubsystem drivetrain;

  // ignore the unused warning, this trajectory is used
  private final Trajectory trajectory;

  public GenerateTrajectory(DriveBaseSubsystem drivetrain, Trajectory trajectory) {
    super(
        trajectory,
        drivetrain::getCtrlsPoseEstimate,
        new RamseteController(
            Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.DriveConstants.ks, Constants.DriveConstants.kv, Constants.DriveConstants.ka),
        Constants.DriveConstants.driveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.kPDriveVelocity, 0, 0),
        new PIDController(Constants.DriveConstants.kPDriveVelocity, 0, 0),
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
