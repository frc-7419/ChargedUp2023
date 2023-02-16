package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

/** Sets the main arm to a specific position using PID controllers. */
public class ArmToSetpoint extends CommandBase {

  private PIDController pidController;
  private ArmSubsystem armSubsystem;
  private double setpoint;

  public ArmToSetpoint(ArmSubsystem armSubsystem, double setpoint) {
    pidController =
        new PIDController(
            Constants.PIDConstants.MainArmKp,
            Constants.PIDConstants.MainArmKi,
            Constants.PIDConstants.MainArmKd);

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
  /**
   * In the execute method, we calculate the power needed to set the main arm to a specific
   * position.
   */
  @Override
  public void execute() {
    
    double mainArmPosition = armSubsystem.getMainPosition();
    double calculatedMainArmPower = pidController.calculate(mainArmPosition);
    armSubsystem.setMainPower(calculatedMainArmPower);

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
