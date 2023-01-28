package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.subsystems.autos.MobilityAuto;
import frc.robot.subsystems.arm.ArmJoystick;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.arm.SmartArm;
import frc.robot.subsystems.drive.GetToTarget;
import frc.robot.subsystems.drive.StraightWithMotionMagic;

public class RobotContainer {
  private final XboxController joystick1 = new XboxController(0);
  private final XboxController joystick2 = new XboxController(1);

  // Subsystems
  private final BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem(gyroSubsystem);
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  // Commands
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick1, driveBaseSubsystem, 0.6, 0.6);
  private final SmartArm smartArm1 = new SmartArm(armSubsystem,ArmConstants.mainArmSetpoint1);
  private final SmartArm smartArm2 = new SmartArm(armSubsystem,ArmConstants.mainArmSetpoint2);
  private final ArmJoystick armJoystick = new ArmJoystick(armSubsystem, joystick1);
  // Autonomous

  // private SendableChooser<Command> autonChooser = new SendableChooser<>();
  // private final GetToTarget getToTarget = new GetToTarget(driveBaseSubsystem, gyroSubsystem);

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick1, Button.kA.value)
        .whileTrue(smartArm1);
    new JoystickButton(joystick1, Button.kB.value)
        .whileTrue(smartArm2);
  }

  private void smartDashboardBindings() {
  }

  private void configureAutoSelector() {
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    armSubsystem.setDefaultCommand(armJoystick);
  }
}
