package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.led.LedSubsystem;

/**
 * Command to arcade drive the robot (left joystick corresponds to straight,
 * right joystick
 * corresponds to turn)
 */
public class ArcadeDrive extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private LedSubsystem ledSubsystem;
  private XboxController joystick;
  private boolean slowMode = false;

  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);

  /**
   * Constructs the ArcadeDrive class
   *
   * @param joystick
   * @param driveBaseSubsystem
   */
  public ArcadeDrive(
      XboxController joystick, DriveBaseSubsystem driveBaseSubsystem, LedSubsystem ledSubsystem) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.ledSubsystem = ledSubsystem;
    addRequirements(driveBaseSubsystem);
  }

  /** Initializes the arcadedrive to reset all motors to default values */
  @Override
  public void initialize() {
    driveBaseSubsystem.factoryResetAll();
    driveBaseSubsystem.setAllDefaultInversions();
    driveBaseSubsystem.coast();
  }

  /** In the execute method, move the joystick to make the robot move. */
  @Override
  public void execute() {
    if (Math.abs(joystick.getLeftY())>0.05 || Math.abs(joystick.getRightX())>0.05){
      driveBaseSubsystem.coast();
    }
    if (joystick.getXButtonPressed()) {
      this.slowMode = !this.slowMode;
    }
    double straightCoefficient = getStraightCoefficient(this.slowMode);
    double joystickInputPower = joystick.getLeftY() * straightCoefficient;
    double xAxisSpeed = speedLimiter.calculate(joystickInputPower);

    double turnCoefficient = getTurnCoefficient(this.slowMode);
    double joystickInputPowerTurn = joystick.getRightX() * turnCoefficient;
    double zAxisRotation = joystickInputPowerTurn;
    driveBaseSubsystem.drive(xAxisSpeed, zAxisRotation);

    // TODO Check if this works
    // driveBaseSubsystem.coast();

    // double leftPower = xAxisSpeed + zAxisRotation;
    // double rightPower = xAxisSpeed - zAxisRotation;

    // driveBaseSubsystem.setLeftPower(leftPower);
    // driveBaseSubsystem.setRightPower(rightPower);
  }

  private double getStraightCoefficient(boolean slowModeOn) {
    if (slowModeOn) {
      ledSubsystem.setLEDOrange();
      return DriveConstants.slowStraight;
    }
    ledSubsystem.setLEDGreen();
    return DriveConstants.driveStraight;
  }

  private double getTurnCoefficient(boolean slowModeOn) {
    if (slowModeOn)
      return DriveConstants.slowTurn;
    return DriveConstants.driveTurn;
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
