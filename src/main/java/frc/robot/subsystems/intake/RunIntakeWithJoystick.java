package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeWithJoystick extends CommandBase{
  private IntakeSubsystem intakeSubsystem;
  private XboxController joystick;
  private double intakeMultiplier;
  
  public RunIntakeWithJoystick(IntakeSubsystem intakeSubsystem, XboxController joystick, double intakeMultiplier) {
    this.intakeSubsystem = intakeSubsystem;
    this.joystick = joystick;
    this.intakeMultiplier = intakeMultiplier;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Math.abs(joystick.getLeftTriggerAxis()) > 0) {
      intakeSubsystem.setPower(-intakeMultiplier * joystick.getLeftTriggerAxis());
    } else if (Math.abs(joystick.getRightTriggerAxis()) > 0) {
      intakeSubsystem.setPower(intakeMultiplier * joystick.getRightTriggerAxis());
    } else {
      intakeSubsystem.setPower(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
      intakeSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }


}