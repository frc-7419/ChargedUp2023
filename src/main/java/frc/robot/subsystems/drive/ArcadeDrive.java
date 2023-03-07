package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;

/**
 * Command to arcade drive the robot (left joystick corresponds to straight, right joystick
 * corresponds to turn)
 */
public class ArcadeDrive extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private XboxController joystick;

  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);
  /**
   * Constructs the ArcadeDrive class
   *
   * @param joystick
   * @param driveBaseSubsystem
   */
  public ArcadeDrive(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
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
    double joystickInputPower = joystick.getLeftY() * DriveConstants.driveStraight;
    double xAxisSpeed = speedLimiter.calculate(joystickInputPower);

    double joystickInputPowerTurn = joystick.getRightX() * DriveConstants.driveTurn;
    double zAxisRotation = joystickInputPowerTurn;
    driveBaseSubsystem.drive(xAxisSpeed, zAxisRotation);

    // TODO Check if this works
    // driveBaseSubsystem.coast();

    // double leftPower = xAxisSpeed + zAxisRotation;
    // double rightPower = xAxisSpeed - zAxisRotation;

    // driveBaseSubsystem.setLeftPower(leftPower);
    // driveBaseSubsystem.setRightPower(rightPower);
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
