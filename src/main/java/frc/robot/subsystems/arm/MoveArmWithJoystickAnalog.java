package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class MoveArmWithJoystickAnalog extends CommandBase {

  private ArmSubsystem armSubsystem;
  private XboxController joystick;
  private WristSubsystem wristSubsystem;
  /**
   * Constructs the class with the parameters.
   *
   * @param armSubsystem
   * @param joystick
   */
  public MoveArmWithJoystickAnalog(ArmSubsystem armSubsystem,WristSubsystem wristSubsystem, XboxController joystick) {
    this.armSubsystem = armSubsystem;
    this.joystick = joystick;
    this.wristSubsystem = wristSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.coast();
  }

  /**
   * Calculates the arm power based on joystick inputs.
   */
  @Override
  public void execute() {
    if (Math.abs(joystick.getLeftY()) <= 0.1){
      armSubsystem.setPower(0);
      armSubsystem.brake();
    } else {
      armSubsystem.coast();
    double joystickArmPower = joystick.getLeftY() * ArmConstants.armPower;
    double armAngle = armSubsystem.getAngle();
    double armFeedforwardPower = ArmConstants.armFeedforward * Math.cos(Math.toRadians(armAngle));
    double armPower = joystickArmPower;
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
