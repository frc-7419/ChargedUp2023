package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GetToTargetVelocityArbitraryFeedforward extends CommandBase {

  private ShooterSubsystem shooterSubsystem;

  private double bKp;
  private double bKi;
  private double bKf;
  private double tKf;

  private double topTargetRawVelocity;
  private double bottomTargetRawVelocity;


  public GetToTargetVelocityArbitraryFeedforward(ShooterSubsystem shooterSubsystem, double topTargetRawVelocity, double bottomTargetRawVelocity, double tKf, double bKf) {
    this.shooterSubsystem = shooterSubsystem;
    this.topTargetRawVelocity = topTargetRawVelocity;
    this.bottomTargetRawVelocity = bottomTargetRawVelocity;
    this.tKf = tKf;
    this.bKf = bKf;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setTopPIDF(0, 0, 0, tKf);
    shooterSubsystem.setBottomPIDF(0, 0, 0, bKf);
    
    // shooterSubsystem.setTopTargetRawVelocity(topTargetRawVelocity);
    // shooterSubsystem.setBottomTargetRawVelocity(bottomTargetRawVelocity);
  }

  @Override
  public void execute() {

    shooterSubsystem.setTopPIDF(0, 0, 0, tKf);
    shooterSubsystem.setBottomPIDF(0, 0, 0, bKf);

    shooterSubsystem.getTopTalon().set(ControlMode.Velocity, topTargetRawVelocity);
    shooterSubsystem.getBottomTalon().set(ControlMode.Velocity, bottomTargetRawVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
