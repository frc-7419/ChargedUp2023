package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmConstants;

public class MoveArmWithJoystickAnalog extends CommandBase {

  private ArmSubsystem armSubsystem;
  private XboxController joystick;
  /**
   * Constructs the class with the parameters.
   *
   * @param armSubsystem
   * @param joystick
   */
  public MoveArmWithJoystickAnalog(ArmSubsystem armSubsystem, XboxController joystick) {
    this.armSubsystem = armSubsystem;
    this.joystick = joystick;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.coast();
  }

  @Override
  public void execute() {
    double joystickArmPower = joystick.getLeftY() * ArmConstants.armPower;
    double armAngle = armSubsystem.getAngle();
    double armFeedforwardPower = ArmConstants.armFeedforward * Math.cos(Math.toRadians(armAngle));
    double armPower = joystickArmPower + armFeedforwardPower;
    // double armPower = joystickArmPower;
    armSubsystem.setPower(armPower);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
