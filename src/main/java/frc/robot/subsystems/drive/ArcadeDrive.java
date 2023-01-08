package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private double kStraight;
  private double kTurn;
  private XboxController joystick;
  
  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);
  // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(70);

  public ArcadeDrive(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem, double kStraight, double kTurn) {
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
    double xSpeed = -speedLimiter.calculate(joystick.getLeftY() * kStraight);
    // double zRotation = rotLimiter.calculate(joystick.getRightX() * kTurn);
    double zRotation = joystick.getRightX() * kTurn;
    driveBaseSubsystem.drive(xSpeed, zRotation);
    
    driveBaseSubsystem.coast();
    
    // double leftPower = kTurn * joystick.getRightX() + kSlowStraight * joystick.getRightY();
    // double rightPower = -kTurn * joystick.getRightX()+ kSlowStraight * joystick.getRightY();

    // double leftPower = xSpeed + zRotation;
    // double rightPower = xSpeed - zRotation;

    double leftPower = xSpeed + zRotation;
    double rightPower = xSpeed - zRotation;

    
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
