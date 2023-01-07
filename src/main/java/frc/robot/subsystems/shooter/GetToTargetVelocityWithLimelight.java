package frc.robot.subsystems.shooter;

import com.team7419.InterpolatedTreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class GetToTargetVelocityWithLimelight extends CommandBase {

  private ShooterSubsystem shooterSubsystem;
  private LimelightSubsystem limelightSubsystem;

  private InterpolatedTreeMap topShooterReferencePoints;
  private InterpolatedTreeMap bottomShooterReferencePoints;

  private double bKp = 0;
  private double tKp = 0;

  private double topTargetVelocity;
  private double bottomTargetVelocity;

  public GetToTargetVelocityWithLimelight(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(shooterSubsystem);

    topShooterReferencePoints = new InterpolatedTreeMap();
    bottomShooterReferencePoints = new InterpolatedTreeMap();

    // config reference points from constants file
    shooterSubsystem.configInterpolatedTreeMapReferencePoints(Constants.kDistanceToTopShooterVelocity, topShooterReferencePoints);
    shooterSubsystem.configInterpolatedTreeMapReferencePoints(Constants.kDistanceToBottomShooterVelocity, bottomShooterReferencePoints);
  }

  @Override
  public void initialize() {
    topTargetVelocity = topShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();
    bottomTargetVelocity = bottomShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();

    shooterSubsystem.setTopPIDF(tKp, 0, 0, 0);
    shooterSubsystem.setBottomPIDF(bKp, 0, 0, 0);

    shooterSubsystem.setTopClosedLoopVelocity(topTargetVelocity);
    shooterSubsystem.setBottomClosedLoopVelocity(bottomTargetVelocity);
  }

  @Override
  public void execute() {
    topTargetVelocity = topShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();
    bottomTargetVelocity = bottomShooterReferencePoints.get(limelightSubsystem.getDistance()).doubleValue();

    shooterSubsystem.setTopPIDF(tKp, 0, 0 , 0);
    shooterSubsystem.setBottomPIDF(bKp, 0, 0, 0);

    shooterSubsystem.setTopClosedLoopVelocity(topTargetVelocity);
    shooterSubsystem.setBottomClosedLoopVelocity(bottomTargetVelocity);
  
    SmartDashboard.putNumber("Top Error", shooterSubsystem.getTopError());
    SmartDashboard.putNumber("Bottom Error", shooterSubsystem.getBottomError());

    // SmartDashboard.putBoolean("Top On Target", shooterSubsystem.topOnTarget());
    // SmartDashboard.putBoolean("Bottom on Target", shooterSubsystem.bottomOnTarget());
    // SmartDashboard.putBoolean("Both on Target", shooterSubsystem.bothOnTarget());
  }

  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.putBoolean("Shooter Running", false);
    shooterSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
