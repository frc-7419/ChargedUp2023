package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.MoveArmWithJoystick;
import frc.robot.subsystems.arm.SmartArm;
import frc.robot.subsystems.arm.SmartExtendedArm;
import frc.robot.subsystems.arm.SmartHome;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.SmartBalanceNew;
import frc.robot.subsystems.wrist.MoveWristWithJoystick;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RobotContainer {
  private final XboxController driverJoystick = new XboxController(0);

  // TODO operatorJoystick is unused since we only need one joystick to test
  private final XboxController operatorJoystick = new XboxController(1);

  // Subsystems
  // TODO will use when testing beambreak
  private final BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();

  // Commands
  private final ArcadeDrive arcadeDrive =
      new ArcadeDrive(driverJoystick, driveBaseSubsystem, 0.2, 0.6);
  private final SmartBalanceNew smartBalanceNew =
      new SmartBalanceNew(driveBaseSubsystem, gyroSubsystem);
  private final SmartArm smartArm1 =
      new SmartArm(armSubsystem, Constants.ArmConstants.mainArmSetpoint1);
  private final SmartArm smartArm2 =
      new SmartArm(armSubsystem, Constants.ArmConstants.mainArmSetpoint2);
  private final SmartHome smartHome = new SmartHome(armSubsystem);
  private final SmartExtendedArm smartExtendedArm = new SmartExtendedArm(armSubsystem, 0);
  private final MoveArmWithJoystick moveArmWithJoystick =
      new MoveArmWithJoystick(armSubsystem, driverJoystick);
  private final MoveWristWithJoystick moveWristWithJoystick = new MoveWristWithJoystick(wristSubsystem, driverJoystick);
  // Autonomous

  // Path Planning Commands

  // TODO will use when testing path planning
  // private final MoveToMid moveToPortal = new MoveToMid(driveBaseSubsystem);

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, Button.kX.value).whileTrue(smartBalanceNew);
    new JoystickButton(driverJoystick, Button.kA.value).whileTrue(smartArm1);
    new JoystickButton(driverJoystick, Button.kB.value).whileTrue(smartArm2);
    new JoystickButton(driverJoystick, Button.kY.value).whileTrue(smartHome);
    new JoystickButton(driverJoystick, Button.kRightBumper.value).whileTrue(smartExtendedArm);
  }

  // TODO update once done with autonomous command
  private void smartDashboardBindings() {}

  // TODO update once done with autonomous command
  private void configureAutoSelector() {}

  public Command getAutonomousCommand() {
    // TODO update once done with autonomous command
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    armSubsystem.setDefaultCommand(moveArmWithJoystick);
    wristSubsystem.setDefaultCommand(moveWristWithJoystick);
  }
}
