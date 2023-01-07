package frc.robot;

import com.team7419.joystick.DoubleButton;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import frc.robot.commands.RunIntakeAndLoaderWithJoystick;
import frc.robot.commands.SmartBalls;
import frc.robot.subsystems.arms.ArmsSubsystem;
import frc.robot.subsystems.arms.AutoClimb;
import frc.robot.subsystems.arms.CoastArms;
import frc.robot.subsystems.arms.RunArmsWithJoystick;
import frc.robot.subsystems.autos.CCCTwoBall;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.MaintainElevatorPosition;
import frc.robot.subsystems.elevator.RunElevatorWithJoystick;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeederWithJoystick;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.DeployIntakeWithJoystick;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.SetLEDColorWithJoystick;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class RobotContainer {
  private final XboxController joystick1 = new XboxController(0);
  private final XboxController joystick2 = new XboxController(1);
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmsSubsystem armsSubsystem = new ArmsSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem();


  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick1, driveBaseSubsystem, 0.6, 0.6);
  // private final SmartShoot smartShoot = new SmartShoot(shooterSubsystem, feederSubsystem, loaderSubsystem, limelightSubsystem, beamBreakSubsystem);
  // auto



  private SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    // align turret
  }

  // private void smartDashboardBindings() {}

  private void configureAutoSelector() {
    // autonChooser.setDefaultOption("two ball", mttdTwoBall);
    // autonChooser.addOption("three ball", mttdThreeBall);
    SmartDashboard.putData(autonChooser);
  }

  public Command getAutonomousCommand() {
    // return oneBallAutoWait;

    // return oneBallAuto;
    // return twoBallAutoExactVelocities;
    // return mttdThreeBall;
    // return cccTwoBallCopy;
    // return twoBallAutoInterpoloation;
    // return threeBallAutoExactVelocities;
    // return threeBallAutoInterpolation;
    // return threeBallAuto;

    return autonChooser.getSelected();
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
  }
}

