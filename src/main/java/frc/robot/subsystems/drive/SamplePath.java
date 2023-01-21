package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.DriveConstants;;

public class SamplePath extends RamseteCommand {
    private final DriveBaseSubsystem drivetrain;
    private final Trajectory trajectory;

    
    public SamplePath(DriveBaseSubsystem drivetrain, Trajectory trajectory) {
        super(
            trajectory,
            drivetrain::getCtrlsPoseEstimate,
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
            DriveConstants.driveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVelocity, 0, 0),
            new PIDController(DriveConstants.kPDriveVelocity, 0, 0),
            drivetrain::tankDriveVolts,
            drivetrain
        );
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
        drivetrain.resetOdometry(new Pose2d());
        super.end(interrupted);
    }
}