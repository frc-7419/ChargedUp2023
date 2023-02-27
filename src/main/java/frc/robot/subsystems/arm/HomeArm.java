package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HomeArm extends CommandBase {

  private PIDController pidController;
  private ArmSubsystem armSubsystem;
  private double homePos;

  public HomeArm(ArmSubsystem armSubsystem) {
    pidController =
        new PIDController(PIDConstants.MainArmKp, PIDConstants.MainArmKi, PIDConstants.MainArmKd);

    this.armSubsystem = armSubsystem;
    this.homePos = armSubsystem.getHomePosition();
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    pidController.setSetpoint(homePos);
    pidController.setTolerance(0.15);
    armSubsystem.coast();
  }

  @Override
  public void execute() {
    double currentArmPosition = armSubsystem.getPosition();
    double pidOutput = pidController.calculate(currentArmPosition);
    armSubsystem.setPower(pidOutput);

    double error = pidController.getPositionError();
    SmartDashboard.putNumber("homeArm Error", error);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setPower(0);
    armSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    boolean setpointReached = pidController.atSetpoint();
    return setpointReached;
  }
}
