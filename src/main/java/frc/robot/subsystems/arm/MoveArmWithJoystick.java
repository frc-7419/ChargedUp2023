package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;

public class MoveArmWithJoystick extends CommandBase {

  private ArmSubsystem armSubsystem;
  private XboxController joystick;
  /**
   * Constructs the class with the parameters.
   * @param armSubsystem
   * @param joystick
   */
  public MoveArmWithJoystick(ArmSubsystem armSubsystem, XboxController joystick) {
    this.armSubsystem = armSubsystem;
    this.joystick = joystick;
    addRequirements(armSubsystem);
  }
  /**
   * Sets the main arm to default to coast
   */
  @Override
  public void initialize() {
    armSubsystem.coastMain();
  }

  /**
   * The execute allows us to control the robot using a joystick
   */
  @Override
  public void execute() {
    double mainPowerJoystick = ArmConstants.mainArmPowerCoefficient * joystick.getRightY();
    double extendedPowerJoystick = ArmConstants.extendedArmPowerCoefficient * joystick.getLeftY();
    armSubsystem.setMainPower(mainPowerJoystick);
    armSubsystem.setExtendedPower(extendedPowerJoystick);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
