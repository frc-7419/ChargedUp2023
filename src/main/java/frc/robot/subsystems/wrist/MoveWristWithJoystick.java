package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Moves wrist up and down with joystick left Y axis */
public class MoveWristWithJoystick extends CommandBase {
  private WristSubsystem wristSubsystem;
  private XboxController joystick;
  private double power;

  public MoveWristWithJoystick(WristSubsystem wristSubsystem, XboxController joystick) {
    this.wristSubsystem = wristSubsystem;
    this.joystick = joystick;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristSubsystem.coast();
  }

  @Override
  public void execute() {
    if (joystick.getLeftBumper()) {
      wristSubsystem.coast();
      wristSubsystem.setPower(-0.2);
    } else if (joystick.getRightBumper()) {
      wristSubsystem.coast();
      wristSubsystem.setPower(0.2);
    } else {
      wristSubsystem.brake();
      wristSubsystem.setPower(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
    wristSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
