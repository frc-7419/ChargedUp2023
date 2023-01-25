package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

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
import frc.robot.subsystems.autos.PathTest;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.SamplePath;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final XboxController joystick1 = new XboxController(0);
  private final XboxController joystick2 = new XboxController(1);

  // Subsystems
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem(gyroSubsystem);
  private final BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem();

  // Commands
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick1, driveBaseSubsystem, 0.6, 0.6);
  // private final SmartShoot smartShoot = new SmartShoot(shooterSubsystem, feederSubsystem, loaderSubsystem, visionSubsystem, beamBreakSubsystem);
  // auto
  private final PathTest pathTest = new PathTest(driveBaseSubsystem);
  // private SendableChooser<Command> autonChooser = new SendableChooser<>();
  // private final GetToTarget getToTarget = new GetToTarget(driveBaseSubsystem, gyroSubsystem);

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    // if (!(Math.abs(joystick1.getLeftY()) > 0.2 || Math.abs(joystick1.getRightY()) > 0.2)) {
      new JoystickButton(joystick1, Button.kRightBumper.value)
          .onTrue(pathTest);
    // }
  }

  private void smartDashboardBindings() {
  }

  private void configureAutoSelector() {
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
    // return new OneBallAuto(driveBaseSubsystem, gyroSubsystem);
    // return pathTest;
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
  }
}
