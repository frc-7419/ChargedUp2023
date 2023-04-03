package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.actions.IntakePiece;
import frc.robot.commands.actions.ScorePiece;
import frc.robot.commands.actions.SmartRetract;
import frc.robot.commands.autos.AutoHigh;
import frc.robot.commands.autos.Balance;
import frc.robot.commands.autos.Mobility;
import frc.robot.commands.autos.MobilityBalance;
import frc.robot.commands.paths.TurnToAngleFieldRelative;
import frc.robot.commands.paths.TurnToAngleRobotRelative;
import frc.robot.constants.NodeConstants;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToSetpointWithFeedforward;
import frc.robot.subsystems.arm.MoveArmWithJoystickAnalog;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpointWithFeedForward;
import frc.robot.subsystems.elevator.MoveElevatorWithJoystickAnalog;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripperWithJoystick;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.SmartBalance;
import frc.robot.subsystems.gyro.TurnWithGyro;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.led.RunLed;
import frc.robot.subsystems.wrist.MoveWristWithJoystick;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToSetpointWithFeedforward;

public class RobotContainer {
  private final XboxController driverJoystick = new XboxController(0);
  private final XboxController operatorJoystick = new XboxController(1);

  // Subsystems
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem(gyroSubsystem);

  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final GripperSubsystem gripperSubsystem = new GripperSubsystem();

  private final LedSubsystem ledSubsystem = new LedSubsystem();

  // // Commands

  private final ArcadeDrive arcadeDrive = new ArcadeDrive(driverJoystick, driveBaseSubsystem, ledSubsystem);

  private final MoveElevatorWithJoystickAnalog moveElevatorWithJoystickAnalog = new MoveElevatorWithJoystickAnalog(
      elevatorSubsystem, operatorJoystick);

  private final MoveArmWithJoystickAnalog moveArmWithJoystickAnalog = new MoveArmWithJoystickAnalog(armSubsystem,
      wristSubsystem, operatorJoystick);

  private final WristToSetpointWithFeedforward wristToSetpointWithFeedforwardReset = new WristToSetpointWithFeedforward(
      wristSubsystem, armSubsystem, NodeState.RESET);
  private final WristToSetpointWithFeedforward wristToSetpointWithFeedforwardLow = new WristToSetpointWithFeedforward(
      wristSubsystem, armSubsystem, NodeState.LOW);
  private final WristToSetpointWithFeedforward wristToSetpointWithFeedforwardHigh = new WristToSetpointWithFeedforward(
      wristSubsystem, armSubsystem, NodeState.HIGH);
  private final WristToSetpointWithFeedforward wristToSetpointWithFeedforwardSubstation = new WristToSetpointWithFeedforward(
      wristSubsystem, armSubsystem, NodeState.SUBSTATION);

  private final SmartBalance smartBalance = new SmartBalance(driveBaseSubsystem,
      gyroSubsystem);

//   private final ArmToSetpoint armToIntakeSetpoint = new ArmToSetpoint(armSubsystem, ArmConstants.intakeSetpoint);

//   private final ArmToSetpoint armToScoreSetpoint = new ArmToSetpoint(armSubsystem, ArmConstants.scoreSetpoint);
private final ArmToSetpointWithFeedforward armToTestSetpoint = new ArmToSetpointWithFeedforward(armSubsystem, NodeState.SUBSTATION);

  // private final ElevatorToSetpoint elevatorToGround =
  // new ElevatorToSetpoint(elevatorSubsystem, NodeState.GROUND);
  // private final ElevatorToSetpoint elevatorToSubstation =
  // new ElevatorToSetpoint(elevatorSubsystem, NodeState.SUBSTATION);
  // private final ElevatorToSetpoint elevatorToLow =
  // new ElevatorToSetpoint(elevatorSubsystem, NodeState.LOW);
  // private final ElevatorToSetpoint elevatorToHigh =
  // new ElevatorToSetpoint(elevatorSubsystem, NodeState.HIGH);

  // private final SmartWrist smartWrist = new SmartWrist(wristSubsystem, 10000);

  // private final RunGripperWithJoystick runGripperWithJoystick =
  // new RunGripperWithJoystick(gripperSubsystem, operatorJoystick, ledSubsystem);

  // private final ElevatorToSetpointWithFeedForward elevatorPIDHigh = new
  // ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.HIGH);
  // private final ElevatorToSetpointWithFeedForward elevatorPIDGround = new
  // ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.RES);
  // private final ElevatorToSetpointWithFeedForward elevatorToReset =
  // new ElevatorToSetpointWithFeedForward(elevatorSubsystem,
  // NodeConstants.NodeState.RESET);

  private final IntakePiece intakePieceSub = new IntakePiece(elevatorSubsystem, armSubsystem, wristSubsystem, NodeState.SUBSTATION);
  private final ElevatorToSetpointWithFeedForward elevatorToLow =
  new ElevatorToSetpointWithFeedForward(elevatorSubsystem,
  NodeConstants.NodeState.LOW);

  private final ElevatorToSetpointWithFeedForward elevatorToHigh =
  new ElevatorToSetpointWithFeedForward(elevatorSubsystem,
  NodeConstants.NodeState.HIGH);
  // private final ElevatorWithMotionMagic elevatorToHigh =
  // new ElevatorWithMotionMagic(elevatorSubsystem,
  // NodeConstants.NodeState.HIGH.elevatorSetpoint);
  // private final ElevatorToSetpoint elevatorToSubstation =
  // new ElevatorToSetpoint(elevatorSubsystem, NodeState.SUBSTATION);
  // private final ElevatorToSetpoint elevatorToLow =
  // new ElevatorToSetpoint(elevatorSubsystem, NodeState.LOW);
  // private final ElevatorToSetpoint elevatorToHigh =
  // new ElevatorToSetpoint(elevatorSubsystem, NodeState.HIGH);
  private final MoveWristWithJoystick moveWristWithJoystick =
  new MoveWristWithJoystick(wristSubsystem, driverJoystick);
  // private final SmartWrist smartWrist = new SmartWrist(wristSubsystem, 10000);
  private final RunGripperWithJoystick runGripperWithJoystick =
  new RunGripperWithJoystick(gripperSubsystem, operatorJoystick, ledSubsystem);
  // private final ElevatorToSetpointWithFeedForward elevatorPIDHigh = new
  // ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.HIGH);
  // private final ElevatorToSetpointWithFeedForward elevatorPIDGround = new
  // ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.GROUND);

  // private final IntakePiece intakePieceGround =
  // new IntakePiece(elevatorSubsystem, armSubsystem, NodeState.RESET);

  // private final IntakePiece intakePieceSubstation =
  // new IntakePiece(elevatorSubsystem, armSubsystem, NodeState.SUBSTATION);

//   private final ScorePiece scorePieceLow = new ScorePiece(elevatorSubsystem, armSubsystem, gripperSubsystem,
//       NodeState.LOW);
  private final ScorePiece scorePieceHigh = new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, NodeState.HIGH);
  private final ScorePiece scorePieceMid = new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem, NodeState.LOW);
  private final IntakePiece intakePieceSingle = new IntakePiece(elevatorSubsystem, armSubsystem, wristSubsystem, NodeState.SINGLE_SUBSTATION);


  private final SmartRetract smartRetract = new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem);

  private final RunLed runLed = new RunLed(ledSubsystem, operatorJoystick);

  // // Autonomous
  private SendableChooser<Command> autonomousChooser = new SendableChooser<>();
//   private final OnePiece onePiece = new OnePiece(driveBaseSubsystem, elevatorSubsystem,
//       armSubsystem, gripperSubsystem);
//   private final TwoPiece twoPiece = new TwoPiece(driveBaseSubsystem, elevatorSubsystem,
//       armSubsystem, gripperSubsystem);
//   private final ThreePiece threePiece = new ThreePiece(driveBaseSubsystem, elevatorSubsystem,
//       armSubsystem, gripperSubsystem);
  // private final Balance balance = new Balance(driveBaseSubsystem, gyroSubsystem);
  // private final Mobility mobility = new Mobility(driveBaseSubsystem);
  private final TurnWithGyro turnWithGyro180 = new TurnWithGyro(driveBaseSubsystem, gyroSubsystem, 180, 0.5);
  // Path Planning Commands

  // TODO will use when testing path planning
  // private final MoveToMid moveToPortal = new MoveToMid(driveBaseSubsystem);
  private final TurnToAngleRobotRelative turn180RobotRelative = new TurnToAngleRobotRelative(driveBaseSubsystem, 180);
  private final TurnToAngleFieldRelative turn180FieldRelative = new TurnToAngleFieldRelative(driveBaseSubsystem, 180);
  private final MobilityBalance mobilityBalance = new MobilityBalance(driveBaseSubsystem, gyroSubsystem);
  public RobotContainer() {
    configureButtonBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, Button.kY.value).whileTrue(smartBalance);
    new JoystickButton(operatorJoystick, Button.kX.value).onTrue(scorePieceHigh);
    new JoystickButton(operatorJoystick, Button.kA.value).onTrue(intakePieceSub);
    new JoystickButton(operatorJoystick, Button.kY.value).onTrue(scorePieceMid);
    new JoystickButton(operatorJoystick, Button.kStart.value).onTrue(intakePieceSingle);
    // new JoystickButton(operatorJoystick,
    // Button.kY.value).onTrue(elevatorToReset);
    // new JoystickButton(driverJoystick,
    // Button.kA.value).whileTrue(turn180RobotRelative);

    // new JoystickButton(operatorJoystick,
    // Button.kLeftBumper.value).onTrue(scorePieceHigh);

    // new JoystickButton(operatorJoystick,
    // Button.kB.value).onTrue(intakePieceGround);
    // new JoystickButton(operatorJoystick,
    // Button.kY.value).onTrue(intakePieceSubstation);

    new JoystickButton(operatorJoystick, Button.kB.value).onTrue(smartRetract);

    // new JoystickButton(operatorJoystick,
    // Button.kRightBumper.value).onTrue(wristToSetpointWithFeedforwardReset);
    // new JoystickButton(operatorJoystick,
    // Button.kLeftBumper.value).onTrue(wristToSetpointWithFeedforwardReset);
    // new JoystickButton(operatorJoy)
    // new JoystickButton(operatorJoystick,
    // Button.kB.value).onTrue(wristToSetpointWithFeedforwardReset);
    // new JoystickButton(operatorJoystick,
    // Button.kY.value).onTrue(wristToSetpointWithFeedforwardLow);
    // new JoystickButton(operatorJoystick,
    // Button.kA.value).onTrue(wristToSetpointWithFeedforwardHigh);
    // new JoystickButton(operatorJoystick, 
    // Button.kX.value).onTrue(wristToSetpointWithFeedforwardSubstation);

    new JoystickButton(driverJoystick, XboxController.Button.kStart.value).onTrue(Commands.parallel(new InstantCommand(armSubsystem::zeroEncoder), new InstantCommand(wristSubsystem::zeroEncoder)));

    new JoystickButton(driverJoystick, XboxController.Button.kB.value).onTrue(turnWithGyro180);
    // new JoystickButton(driverJoystick, XboxController.Button.kX.value).onTrue(new InstantCommand(driveBaseSubsystem::brake));

    // new JoystickButton(operatorJoystick, XboxController.Button.kX.value).onTrue(new ElevatorWithMotionMagic(elevatorSubsystem, 150000));
    // new JoystickButton(operatorJoystick, XboxController.Button.kB.value).onTrue(new ElevatorWithMotionMagic(elevatorSubsystem, 7500));
    // new JoystickButton(operatorJoystick, XboxController.Button.kX.value).onTrue(new ElevatorWithMotionMagic(elevatorSubsystem, 320000));
  }

  private void configureAutoSelector() {
    // autonomousChooser.setDefaultOption("Mobility", mobility);
    // autonomousChooser.setDefaultOption("Balance", balance);
    // autonomousChooser.addOption("One Piece", onePiece);
    // autonomousChooser.addOption("Two Piece", twoPiece);
    // autonomousChooser.addOption("Three Piece", threePiece);
    SmartDashboard.putData(autonomousChooser);
  }

  public Command getAutonomousCommand() {
    // ledSubsystem.rainbowLED(0);
    // return autonomousChooser.getSelected();
    // return balance;
    // return new Mobility(driveBaseSubsystem);
    // return mobility;xxxxxxxxxxxxxxxxx
    // return new MobilityBalance(driveBaseSubsystem, gyroSubsystem);
    // return new AutoHigh(driveBaseSubsystem, elevatorSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, gyroSubsystem, NodeState.HIGH);
    return new Balance(driveBaseSubsystem, gyroSubsystem);
    // return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    gripperSubsystem.setDefaultCommand(runGripperWithJoystick);
    wristSubsystem.setDefaultCommand(moveWristWithJoystick);
    armSubsystem.setDefaultCommand(moveArmWithJoystickAnalog);
    elevatorSubsystem.setDefaultCommand(moveElevatorWithJoystickAnalog);
    // ledSubsystem.setDefaultCommand(runLed);
  }

  public void zeroSensor(String allianceColor, String allianceSide){
    
    gyroSubsystem.zeroPitch();
    gyroSubsystem.zeroRoll();
    gyroSubsystem.zeroYaw(allianceColor);
    elevatorSubsystem.zeroEncoder();
    armSubsystem.zeroEncoder();
    wristSubsystem.zeroEncoder();
  }
  public void zeroSensor(){
    gyroSubsystem.zeroPitch();
    gyroSubsystem.zeroRoll();
    elevatorSubsystem.zeroEncoder();
    armSubsystem.zeroEncoder();
    wristSubsystem.zeroEncoder();
  }
}
