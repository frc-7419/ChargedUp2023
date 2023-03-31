package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class MoveArmWithJoystickAnalog extends CommandBase {

  private ArmSubsystem armSubsystem;
  private XboxController joystick;
  private WristSubsystem wristSubsystem;
  private ArmFeedforward armFeedforward;
  /**
   * Constructs the class with the parameters.
   *
   * @param armSubsystem
   * @param joystick
   */
  public MoveArmWithJoystickAnalog(
      ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, XboxController joystick) {
    this.armSubsystem = armSubsystem;
    this.joystick = joystick;
    this.wristSubsystem = wristSubsystem;
    armFeedforward = ArmConstants.armFeedforward;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.coast();
  }

  /** Calculates the arm power based on joystick inputs. */
  @Override
  public void execute() {
    if (Math.abs(joystick.getLeftY()) <= 0.1) {
      armSubsystem.setPower(0);
      armSubsystem.brake();
    } else {
      armSubsystem.coast();
      double joystickArmPower = joystick.getLeftY() * ArmConstants.armPower;
      double armAngle = armSubsystem.getAngle();
      double armVelocity = armSubsystem.getVelocityInRadians();
      // double armFeedforwardPower = Math.copySign(armFeedforward.calculate(Units.degreesToRadians(armAngle), 0), joystickArmPower);
      double armFeedforwardPower = Math.copySign(0.08 * Math.cos(Units.degreesToRadians(armAngle)), joystickArmPower);
      double armPower = joystickArmPower + armFeedforwardPower;
      armSubsystem.setPower(armPower);
      // wristSubsystem.setPower(0.5*joystickArmPower);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
