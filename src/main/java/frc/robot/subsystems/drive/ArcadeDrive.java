package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
/**
 * Command to arcade drive the robot (left joystick --> straight, right joystick --> turn)
 */
public class ArcadeDrive extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private double kStraight;
  private double kTurn;
  private XboxController joystick;

  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);

  public ArcadeDrive(
      XboxController joystick,
      DriveBaseSubsystem driveBaseSubsystem,
      double kStraight,
      double kTurn) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.kStraight = kStraight;
    this.kTurn = kTurn;
    addRequirements(driveBaseSubsystem);
  }

  @Override
  public void initialize() {
    driveBaseSubsystem.factoryResetAll();
    driveBaseSubsystem.setAllDefaultInversions();
    driveBaseSubsystem.coast();
  }

  @Override
  public void execute() {
    double joystickInputPower = joystick.getLeftY() * kStraight;
    double xAxisSpeed = -speedLimiter.calculate(joystickInputPower);

    double joystickInputPowerTurn = joystick.getRightX() * kTurn;
    double zAxisRotation = joystickInputPowerTurn;
    driveBaseSubsystem.drive(xAxisSpeed, zAxisRotation);

    driveBaseSubsystem.coast();

    double leftPower = xAxisSpeed + zAxisRotation;
    double rightPower = xAxisSpeed - zAxisRotation;

    driveBaseSubsystem.setLeftPower(leftPower);
    driveBaseSubsystem.setRightPower(rightPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setAllPower(0);
  }
}
