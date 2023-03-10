package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.actions.IntakePiece;
import frc.robot.commands.actions.ScorePiece;
import frc.robot.commands.actions.SmartRetract;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.MoveArmWithJoystickAnalog;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.MoveElevatorWithJoystickAnalog;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripperWithJoystick;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.SmartBalance;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.led.RunLed;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RobotContainer {
  private final XboxController driverJoystick = new XboxController(0);
  private final XboxController operatorJoystick = new XboxController(1);

  // Subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
  private final LedSubsystem ledSubsystem = new LedSubsystem();

  // // Commands

  private final ArcadeDrive arcadeDrive = new ArcadeDrive(driverJoystick, driveBaseSubsystem);
  private final SmartBalance smartBalance = new SmartBalance(driveBaseSubsystem, gyroSubsystem);

  // // private final ArmToSetpoint armToIntakeSetpoint =
  // //     new ArmToSetpoint(armSubsystem, ArmConstants.intakeSetpoint);

  // // private final ArmToSetpoint armToScoreSetpoint =
  // //     new ArmToSetpoint(armSubsystem, ArmConstants.scoreSetpoint);

  // // //   private final ElevatorToSetpoint elevatorToGround =
  // // //       new ElevatorToSetpoint(elevatorSubsystem, NodeState.GROUND);
  // // //   private final ElevatorToSetpoint elevatorToSubstation =
  // // //       new ElevatorToSetpoint(elevatorSubsystem, NodeState.SUBSTATION);
  // // //   private final ElevatorToSetpoint elevatorToLow =
  // // //       new ElevatorToSetpoint(elevatorSubsystem, NodeState.LOW);
  // // //   private final ElevatorToSetpoint elevatorToHigh =
  // // //       new ElevatorToSetpoint(elevatorSubsystem, NodeState.HIGH);
  // // private final MoveWristWithJoystick moveWristWithJoystick =
  // //     new MoveWristWithJoystick(wristSubsystem, driverJoystick);
  // // // private final SmartWrist smartWrist = new SmartWrist(wristSubsystem, 10000);

  private final RunGripperWithJoystick runGripperWithJoystick =
      new RunGripperWithJoystick(gripperSubsystem, operatorJoystick, ledSubsystem);

  // private final ElevatorToSetpointWithFeedForward elevatorPIDHigh = new
  // ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.HIGH);
  // private final ElevatorToSetpointWithFeedForward elevatorPIDGround = new
  // ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.RES);

  // //   private final ElevatorToSetpoint elevatorToGround =
  // //       new ElevatorToSetpoint(elevatorSubsystem, NodeState.GROUND);
  // //   private final ElevatorToSetpoint elevatorToSubstation =
  // //       new ElevatorToSetpoint(elevatorSubsystem, NodeState.SUBSTATION);
  // //   private final ElevatorToSetpoint elevatorToLow =
  // //       new ElevatorToSetpoint(elevatorSubsystem, NodeState.LOW);
  // //   private final ElevatorToSetpoint elevatorToHigh =
  // //       new ElevatorToSetpoint(elevatorSubsystem, NodeState.HIGH);
  // private final MoveWristWithJoystick moveWristWithJoystick =
  //     new MoveWristWithJoystick(wristSubsystem, driverJoystick);
  // // private final SmartWrist smartWrist = new SmartWrist(wristSubsystem, 10000);
  // private final RunGripperWithJoystick runGripperWithJoystick =
  //     new RunGripperWithJoystick(gripperSubsystem, operatorJoystick);
  // private final ElevatorToSetpointWithFeedForward elevatorPIDHigh = new
  // ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.HIGH);
  // private final ElevatorToSetpointWithFeedForward elevatorPIDGround = new
  // ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.GROUND);
  private final MoveElevatorWithJoystickAnalog moveElevatorWithJoystickAnalog =
      new MoveElevatorWithJoystickAnalog(elevatorSubsystem, operatorJoystick);
  private final MoveArmWithJoystickAnalog moveArmWithJoystickAnalog =
      new MoveArmWithJoystickAnalog(armSubsystem, operatorJoystick);
  private final IntakePiece intakePieceGround =
      new IntakePiece(elevatorSubsystem, armSubsystem, NodeState.RESET);

  private final IntakePiece intakePieceSubstation =
      new IntakePiece(elevatorSubsystem, armSubsystem, NodeState.SUBSTATION);

  private final ScorePiece scorePieceLow =
      new ScorePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.LOW);
  private final ScorePiece scorePieceHigh =
      new ScorePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.HIGH);

  private final SmartRetract smartRetract =
      new SmartRetract(elevatorSubsystem, armSubsystem, gripperSubsystem);

  private final RunLed runLed = new RunLed(ledSubsystem, operatorJoystick);

  // Autonomous
  private SendableChooser<Command> autonomousChooser = new SendableChooser<>();
  // Path Planning Commands

  // TODO will use when testing path planning
  // private final MoveToMid moveToPortal = new MoveToMid(driveBaseSubsystem);

  public RobotContainer() {
    configureButtonBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    // new JoystickButton(driverJoystick, Button.kX.value).whileTrue(smartBalanceNew);
    // new JoystickButton(driverJoystick, Button.kA.value).whileTrue(armToIntakeSetpoint);
    // new JoystickButton(driverJoystick, Button.kB.value).whileTrue(armToScoreSetpoint);
    // new JoystickButton(driverJoystick, Button.kLeftBumper.value).whileTrue(smartWrist);
    // new JoystickButton(driverJoystick, Button.kRightBumper.value).whileTrue(elevatorToSetpoint);
    // new JoystickButton(operatorJoystick, Button.kX.value).whileTrue(elevatorToGround);
    // new JoystickButton(operatorJoystick, Button.kRightBumper.value).whileTrue(elevatorToHigh);
    // new JoystickButton(operatorJoystick, Button.kLeftBumper.value).whileTrue(elevatorToLow);
    // new JoystickButton(operatorJoystick, Button.kRightBumper.value).onTrue(scorePieceLow);
    // new JoystickButton(operatorJoystick, Button.kLeftBumper.value).onTrue(scorePieceHigh);

    // new JoystickButton(operatorJoystick, Button.kB.value).onTrue(intakePieceGround);
    // new JoystickButton(operatorJoystick, Button.kY.value).onTrue(intakePieceSubstation);

    new JoystickButton(operatorJoystick, Button.kB.value).onTrue(intakePieceGround);
    new JoystickButton(operatorJoystick, Button.kY.value).onTrue(intakePieceSubstation);

    new JoystickButton(operatorJoystick, Button.kA.value).onTrue(smartRetract);
  }

  private void configureAutoSelector() {
    autonomousChooser.setDefaultOption("", null);
    autonomousChooser.addOption("", null);
    autonomousChooser.addOption("", null);
    SmartDashboard.putData(autonomousChooser);
  }

  public Command getAutonomousCommand() {
    ledSubsystem.rainbowLED(0);
    return autonomousChooser.getSelected();
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    armSubsystem.setDefaultCommand(moveArmWithJoystickAnalog);
    // wristSubsystem.setDefaultCommand(moveWristWithJoystick);
    elevatorSubsystem.setDefaultCommand(moveElevatorWithJoystickAnalog);
    gripperSubsystem.setDefaultCommand(runGripperWithJoystick);
    ledSubsystem.setDefaultCommand(runLed);
  }
}
