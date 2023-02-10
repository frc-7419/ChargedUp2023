package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SmartHome extends CommandBase {

  private PIDController pidController;
  private ArmSubsystem armSubsystem;
  private double initialMainArmPosition;

  public SmartHome(ArmSubsystem armSubsystem) {
    pidController =
        new PIDController(
          PIDConstants.MainArmKp, 
          PIDConstants.MainArmKi, 
          PIDConstants.MainArmKd);
          
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    initialMainArmPosition = armSubsystem.getHomePosition();
    pidController.setSetpoint(initialMainArmPosition);
    pidController.setTolerance(0.15);
    armSubsystem.coastMain();
  }

  
  /**
   * Uses PID controller to set the arm to the original position (when first initialized)
   */
  @Override
  public void execute() {
    double mainArmPosition = armSubsystem.getMainPosition();
    double calculatedArmPower = pidController.calculate(mainArmPosition);
    armSubsystem.setMainPower(calculatedArmPower);
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
