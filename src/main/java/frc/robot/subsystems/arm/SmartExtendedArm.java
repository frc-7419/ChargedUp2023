package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Returns the main and extended arm to a setpoint
 */
public class SmartExtendedArm extends CommandBase {

  private PIDController extendedArmPIDController;
  private PIDController mainArmController;
  private ArmSubsystem armSubsystem;
  private double setpoint;
  private double mainArmSetPoint;

  /**
   * Construct the class with the parameters. Also sets a new PID controller for the main arm and the extended arm. 
   * @param armSubsystem
   * @param setpoint
   */
  public SmartExtendedArm(ArmSubsystem armSubsystem, double setpoint) {

    extendedArmPIDController =
        new PIDController(
            PIDConstants.ExtendedArmKp, PIDConstants.ExtendedArmKi, PIDConstants.ExtendedArmKd);

    mainArmController =
        new PIDController(
            PIDConstants.MainArmKp, 
            PIDConstants.MainArmKi, 
            PIDConstants.MainArmKd);

    this.armSubsystem = armSubsystem;
    this.setpoint = setpoint;
    addRequirements(armSubsystem);
  }

  /**
   * Sets the setpoints in for the PID controller, as well as a tolerance for the PID controller
   */
  @Override
  public void initialize() {
    this.mainArmSetPoint = armSubsystem.getMainPosition();
    extendedArmPIDController.setSetpoint(mainArmSetPoint);
    extendedArmPIDController.setTolerance(0.15);
    mainArmController.setSetpoint(setpoint);
    mainArmController.setTolerance(0.15);
    armSubsystem.coastExtended();
    armSubsystem.coastMain();
  }
  /**
   * The method uses the setpoints that were set in the initialize method as well as the PID controllers 
   * to calculate how much power will be needed to run the arms to the set positions
   */
  @Override
  public void execute() {
    double armMainPosition = armSubsystem.getMainPosition();
    double calculatedMainArmPower = mainArmController.calculate(armMainPosition);
    armSubsystem.setMainPower(calculatedMainArmPower);
    double extendedArmAngle = armSubsystem.getExtendedAngle();
    double calculatedExtendedArmPower = extendedArmPIDController.calculate(extendedArmAngle);
    armSubsystem.setExtendedPower(calculatedExtendedArmPower);

    SmartDashboard.putNumber("Extended Arm Error", extendedArmPIDController.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setExtendedPower(0);
    armSubsystem.brakeExtended();
    armSubsystem.brakeMain();
  }

  /**
   * When command is finished, return if set point error 
   * of extendedArmPIDController is within acceptable bounds.
   */
  @Override
  public boolean isFinished() {
    return extendedArmPIDController.atSetpoint();
  }
}
