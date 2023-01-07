package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class AlignTurret extends CommandBase {

  private TurretSubsystem turretSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private PIDController pidController;
  
  private double kP, kI, kD;

  private double pidOutput;
  private double tx;
  private double tv;

  public AlignTurret(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    pidController = new PIDController(PIDConstants.TurretKp, 0, 0);
    pidController.setSetpoint(0);
    pidController.setTolerance(1);
  }

  @Override
  public void execute() {
    tx = limelightSubsystem.getTx();
    tv = limelightSubsystem.getTv();

    if (tv == 1.0) {
      pidController = new PIDController(PIDConstants.TurretKp, 0, 0);
      pidOutput = pidController.calculate(tx);
      turretSubsystem.setPower(-pidOutput);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
 