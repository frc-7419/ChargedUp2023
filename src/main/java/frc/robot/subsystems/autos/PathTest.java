package frc.robot.subsystems.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SamplePath;

public class PathTest extends SequentialCommandGroup {
    public PathTest(DriveBaseSubsystem driveBaseSubsystem) {
        addCommands(
           new SamplePath(driveBaseSubsystem, PathPlanner.loadPath("Test Path", new PathConstraints(Constants.DriveConstants.maxVelocity, Constants.DriveConstants.maxAcceleration)))
        );
        
    }
}