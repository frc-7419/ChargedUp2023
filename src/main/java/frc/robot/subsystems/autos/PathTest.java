package frc.robot.subsystems.autos;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SamplePath;

public class PathTest extends SequentialCommandGroup {
    public PathTest(DriveBaseSubsystem driveBaseSubsystem) {
        List<PathPoint> waypoints = new ArrayList<PathPoint>();
        waypoints.add(new PathPoint(new Translation2d(6.91, 2.68), Rotation2d.fromDegrees(180)));
        waypoints.add(new PathPoint(new Translation2d(6.32, 1.53), Rotation2d.fromDegrees(90)));
        waypoints.add(new PathPoint(new Translation2d(5.09, 2.9), Rotation2d.fromDegrees(180)));
        addCommands(
        //    new SamplePath(driveBaseSubsystem, PathPlanner.loadPath("Mid", new PathConstraints(Constants.DriveConstants.maxVelocity, Constants.DriveConstants.maxAcceleration)))
        new SamplePath(driveBaseSubsystem, PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints)),
        new SamplePath(driveBaseSubsystem, PathPlanner.loadPath("Mid", PathPlanner.getConstraintsFromPath("Mid")))
        );
        
    }
}