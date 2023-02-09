package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmJoystick;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.SmartArm;
import frc.robot.subsystems.arm.SmartExtendedArm;
import frc.robot.subsystems.arm.SmartHome;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;

public class RobotContainer {
  private final XboxController driverJoystick = new XboxController(0);
  // private final XboxController operatorJoystick = new XboxController(1);

  // Subsystems
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem(gyroSubsystem);
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  // Commands
  private final ArcadeDrive arcadeDrive =
      new ArcadeDrive(driverJoystick, driveBaseSubsystem, 0.6, 0.6);
  private final SmartArm smartArm1 = new SmartArm(armSubsystem, ArmConstants.mainArmSetpoint1);
  private final SmartArm smartArm2 = new SmartArm(armSubsystem, ArmConstants.mainArmSetpoint2);
  private final SmartHome smartHome = new SmartHome(armSubsystem);
  private final SmartExtendedArm smartExtendedArm = new SmartExtendedArm(armSubsystem, 0);
  private final ArmJoystick armJoystick = new ArmJoystick(armSubsystem, driverJoystick);
  // Autonomous

  // TODO implement autonomous chooser once autonomous routines are finalized
  // private SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, Button.kA.value).whileTrue(smartArm1);
    new JoystickButton(driverJoystick, Button.kB.value).whileTrue(smartArm2);
    new JoystickButton(driverJoystick, Button.kY.value).whileTrue(smartHome);
    new JoystickButton(driverJoystick, Button.kRightBumper.value).whileTrue(smartExtendedArm);
  }

  private void smartDashboardBindings() {}

  private void configureAutoSelector() {}

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }


  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    armSubsystem.setDefaultCommand(armJoystick);
  }
}
