package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmJoystick extends CommandBase {
  
  private ArmSubsystem armSubsystem;
  private XboxController joystick;

  public ArmJoystick(ArmSubsystem armSubsystem, XboxController joystick) {
    this.armSubsystem = armSubsystem;
    this.joystick = joystick;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armSubsystem.setMainPower(joystick.getRightY());
    armSubsystem.setExtendedPower(0.5*joystick.getLeftY());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
