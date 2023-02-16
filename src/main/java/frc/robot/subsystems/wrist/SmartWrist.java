package frc.robot.subsystems.wrist;

import static frc.robot.Constants.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Uses PID to move wrist to a setpoint */
public class SmartWrist extends CommandBase {
  private WristSubsystem wristSubsystem;
  private PIDController wristController;
  private double setpoint;

  public SmartWrist(WristSubsystem wristSubsystem, double setpoint) {
    this.wristSubsystem = wristSubsystem;
    this.setpoint = setpoint;
    wristController =
        new PIDController(PIDConstants.wristkP, PIDConstants.wristkI, PIDConstants.wristkD);
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristController.setSetpoint(setpoint);
    wristController.setTolerance(PIDConstants.wristTolerance);
  }

  @Override
  public void execute() {
    double wristPosition = wristSubsystem.getPosition();
    double output = wristController.calculate(wristPosition);
    wristSubsystem.setPower(output);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
    wristSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return wristController.atSetpoint();
  }
}
