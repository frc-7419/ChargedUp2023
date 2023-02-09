package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SmartArm extends CommandBase {

  private PIDController pidController;
  private ArmSubsystem armSubsystem;
  private double setpoint;

  public SmartArm(ArmSubsystem armSubsystem, double setpoint) {
    pidController =
        new PIDController(
        PIDConstants.MainArmKp,
        PIDConstants.MainArmKi, 
        PIDConstants.MainArmKd);
    this.armSubsystem = armSubsystem;
    this.setpoint = setpoint;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    pidController.setSetpoint(setpoint);
    pidController.setTolerance(0.15);
    armSubsystem.coastMain();
  }

  @Override
  public void execute() {
    armSubsystem.setMainPower(pidController.calculate(armSubsystem.getMainPosition()));
    SmartDashboard.putNumber("Arm Error", pidController.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setMainPower(0);
    armSubsystem.brakeMain();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
