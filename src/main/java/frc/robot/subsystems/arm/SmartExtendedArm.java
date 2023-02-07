package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;

public class SmartExtendedArm extends CommandBase {
  
  private PIDController extendedArmPIDController;
  private PIDController mainArmController;
  private ArmSubsystem armSubsystem;
  private double setpoint;
  private double mainArmSetPoint;
  public SmartExtendedArm(ArmSubsystem armSubsystem, double setpoint) {
    extendedArmPIDController = new PIDController(PIDConstants.ExtendedArmKp, PIDConstants.ExtendedArmKi, PIDConstants.ExtendedArmKd);
    mainArmController = new PIDController(PIDConstants.MainArmKp, PIDConstants.MainArmKi, PIDConstants.MainArmKd);
    this.armSubsystem = armSubsystem;
    this.setpoint = setpoint;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    this.mainArmSetPoint = armSubsystem.getMain2Position();
    extendedArmPIDController.setSetpoint(mainArmSetPoint);
    extendedArmPIDController.setTolerance(0.15);
    mainArmController.setSetpoint(setpoint);
    mainArmController.setTolerance(0.15);
    armSubsystem.coastExtended();
    armSubsystem.coastMain();
  }

  @Override
  public void execute() {
    armSubsystem.setMainPower(mainArmController.calculate(armSubsystem.getMain2Position()));
    armSubsystem.setExtendedPower(extendedArmPIDController.calculate(armSubsystem.getExtendedAngle()));
    SmartDashboard.putNumber("extended arm error", extendedArmPIDController.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setExtendedPower(0);
    armSubsystem.brakeExtended();
    armSubsystem.brakeMain();
  }

  @Override
  public boolean isFinished() {
    return extendedArmPIDController.atSetpoint();
  }
}
