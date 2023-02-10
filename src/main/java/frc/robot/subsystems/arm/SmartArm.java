package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

/**
 * 
 */
public class SmartArm extends CommandBase {

  private PIDController pidController;
  private ArmSubsystem armSubsystem;
  private double setpoint;
  /**
   * Construct the class with the parameters. Also sets a new PID controller for the main arm.
   * @param armSubsystem
   * @param setpoint
   */
  public SmartArm(ArmSubsystem armSubsystem, double setpoint) {
    pidController =
        new PIDController(
        Constants.PIDConstants.MainArmKp,
        Constants.PIDConstants.MainArmKi, 
        Constants.PIDConstants.MainArmKd);
        
    this.armSubsystem = armSubsystem;
    this.setpoint = setpoint;
    addRequirements(armSubsystem);
  }
  /**
   * Set the set points for the PID controller as well as the tolerance. Also keeps the armsystem at coast at default.
   */
  @Override
  public void initialize() {
    pidController.setSetpoint(setpoint);
    pidController.setTolerance(0.15);
    armSubsystem.coastMain();
  }
  /**
   * 
   */
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
