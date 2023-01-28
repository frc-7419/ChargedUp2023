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
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SamplePath;

public class PathTest extends SequentialCommandGroup {
    public PathTest(DriveBaseSubsystem driveBaseSubsystem) {
        List<PathPoint> waypoints = new ArrayList<PathPoint>();
        waypoints.add(new PathPoint(new Translation2d(driveBaseSubsystem.getCtrlsPoseEstimate().getX(), driveBaseSubsystem.getCtrlsPoseEstimate().getY()), Rotation2d.fromDegrees(driveBaseSubsystem.getCtrlsPoseEstimate().getRotation().getDegrees())));
        waypoints.add(new PathPoint(new Translation2d(12.58, 0.91), Rotation2d.fromDegrees(0)));
        // the last way point ^ needs to be the start of the pathplanning path - note from kaival keshav braden and robot
        addCommands(
        //    new SamplePath(driveBaseSubsystem, PathPlanner.loadPath("Mid", new PathConstraints(Constants.DriveConstants.maxVelocity, Constants.DriveConstants.maxAcceleration)))
        new SamplePath(driveBaseSubsystem, PathPlanner.generatePath(new PathConstraints(2, 0.5), waypoints)),
        new SamplePath(driveBaseSubsystem, PathPlanner.loadPath("BlueMid", PathPlanner.getConstraintsFromPath("BlueMid")))
        );
        
    }
}