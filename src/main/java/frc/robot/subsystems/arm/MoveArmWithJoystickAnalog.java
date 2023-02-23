package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
  /** Sets the main arm to default to coast */
  @Override
  public void initialize() {
    armSubsystem.coastMain();
  }

  /** The execute allows us to control the robot using a joystick */
  @Override
  public void execute() {
    armSubsystem.setMainPower(joystick.getLeftY() * 0.7);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
