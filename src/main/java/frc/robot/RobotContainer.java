package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.paths.MoveToPortal;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;

public class RobotContainer {
  private final XboxController driverJoystick = new XboxController(0);

  // TODO operatorJoystick is unused since we only need one joystick to test
  private final XboxController operatorJoystick = new XboxController(1);

  // Subsystems
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem(gyroSubsystem);

  // Commands
  private final ArcadeDrive arcadeDrive =
      new ArcadeDrive(driverJoystick, driveBaseSubsystem, 0.6, 0.6);

  // Autonomous Commands

  // Path Planning Commands
  private final MoveToPortal moveToPortal = new MoveToPortal(driveBaseSubsystem);

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, Button.kRightBumper.value).onTrue(moveToPortal);
  }

  private void smartDashboardBindings() {}

  private void configureAutoSelector() {}

  public Command getAutonomousCommand() {
    // TODO placeholder until we get actual autonomous commands
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
  }
}
