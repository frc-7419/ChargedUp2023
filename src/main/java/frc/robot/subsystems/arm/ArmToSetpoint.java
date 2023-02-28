package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.PIDConstants;

/** Sets the main arm to a specific position using PID controllers. */
public class ArmToSetpoint extends CommandBase {

  private PIDController pidController;
  private ArmSubsystem armSubsystem;
  private double setpoint;

  public ArmToSetpoint(ArmSubsystem armSubsystem, double setpoint) {
    pidController =
        new PIDController(PIDConstants.MainArmKp, PIDConstants.MainArmKi, PIDConstants.MainArmKd);

    this.armSubsystem = armSubsystem;
    this.setpoint = setpoint;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    pidController.setSetpoint(setpoint);
    pidController.setTolerance(0.15);
    armSubsystem.coast();
  }
  /**
   * In the execute method, we calculate the power needed to set the main arm to a specific
   * position.
   */
  @Override
  public void execute() {

    double mainArmPosition = armSubsystem.getPosition();
    double calculatedMainArmPower = pidController.calculate(mainArmPosition);
    armSubsystem.setPower(calculatedMainArmPower);

    double error = pidController.getPositionError();
    SmartDashboard.putNumber("armToSetpoint Error", error);
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
