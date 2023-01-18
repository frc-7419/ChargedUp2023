// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class SamplePath extends CommandBase {
  /** Creates a new SamplePath. */
  private DriveBaseSubsystem driveBaseSubsystem;
  private PathPlannerTrajectory sampleTrajectory;
  private Timer timer;
  private PIDController pidControllerX;
  private PIDController pidControllerY;
  private PIDController pidControllerGyro;
  private PPHolonomicDriveController driveController;
  public SamplePath(DriveBaseSubsystem driveBaseSubsystem) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    timer = new Timer();
    //arbitrary pid constants for now
    pidControllerX = new PIDController(0, 0, 0);
    pidControllerY = new PIDController(0, 0, 0);
    pidControllerGyro = new PIDController(0, 0, 0);
    driveController = new PPHolonomicDriveController(pidControllerX, pidControllerY, pidControllerGyro);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sampleTrajectory = PathPlanner.loadPath("Test Path", new PathConstraints(4,3));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerState nextSetPoint = (PathPlannerState)(sampleTrajectory.sample(timer.get() + 0.02));
    Pose2d curPose = driveBaseSubsystem.getCtrlsPoseEstimate();
    driveBaseSubsystem.driveWithChassisSpeeds(driveController.calculate(curPose, nextSetPoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (sampleTrajectory.getTotalTimeSeconds() < timer.get() && driveController.atReference());
  }
}
