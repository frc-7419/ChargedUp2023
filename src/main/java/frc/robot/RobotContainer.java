package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.actions.IntakePiece;
import frc.robot.commands.actions.ScorePiece;
import frc.robot.commands.actions.SmartRetract;
import frc.robot.constants.ElevatorConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.MoveArmWithJoystickAnalog;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpointWithFeedForward;
import frc.robot.subsystems.elevator.MoveElevatorWithJoystickAnalog;
import frc.robot.subsystems.gripper.GripperSubsystem;

public class RobotContainer {
  private final XboxController driverJoystick = new XboxController(0);

  // TODO operatorJoystick is unused since we only need one joystick to test
  private final XboxController operatorJoystick = new XboxController(1);

  // Subsystems
  // TODO will use when testing beambreak
  // private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final GripperSubsystem gripperSubsystem = new GripperSubsystem();

  // Commandsq
  // TODO will use when driving
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(driverJoystick, driveBaseSubsystem);
  // private final BalanceOnChargeStationNew smartBalanceNew =
  //     new BalanceOnChargeStationNew(driveBaseSubsystem, gyroSubsystem);

  // private final ArmToSetpoint armToIntakeSetpoint =
  //     new ArmToSetpoint(armSubsystem, ArmConstants.intakeSetpoint);

  // private final ArmToSetpoint armToScoreSetpoint =
  //     new ArmToSetpoint(armSubsystem, ArmConstants.scoreSetpoint);

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
  private final ElevatorToSetpointWithFeedForward elevatorPIDHigh =
      new ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.HIGH);
  private final ElevatorToSetpointWithFeedForward elevatorPIDGround =
      new ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.GROUND);
  private final MoveElevatorWithJoystickAnalog moveElevatorWithJoystickAnalog =
      new MoveElevatorWithJoystickAnalog(elevatorSubsystem, operatorJoystick);
  private final MoveArmWithJoystickAnalog moveArmWithJoystickAnalog =
      new MoveArmWithJoystickAnalog(armSubsystem, operatorJoystick);
  private final IntakePiece intakePieceGround =
      new IntakePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.GROUND);
  private final IntakePiece intakePieceSubstation =
      new IntakePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.SUBSTATION);

  private final ScorePiece scorePieceLow =
      new ScorePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.LOW);
  private final ScorePiece scorePieceHigh =
      new ScorePiece(elevatorSubsystem, armSubsystem, gripperSubsystem, NodeState.HIGH);

  private final SmartRetract smartRetract =
      new SmartRetract(elevatorSubsystem, armSubsystem, gripperSubsystem);

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
    // new JoystickButton(driverJoystick, Button.kX.value).whileTrue(smartBalanceNew);
    // new JoystickButton(driverJoystick, Button.kA.value).whileTrue(armToIntakeSetpoint);
    // new JoystickButton(driverJoystick, Button.kB.value).whileTrue(armToScoreSetpoint);
    // new JoystickButton(driverJoystick, Button.kLeftBumper.value).whileTrue(smartWrist);
    // new JoystickButton(driverJoystick, Button.kRightBumper.value).whileTrue(elevatorToSetpoint);
    // new JoystickButton(operatorJoystick, Button.kX.value).whileTrue(elevatorToGround);
    // new JoystickButton(operatorJoystick, Button.kRightBumper.value).whileTrue(elevatorToHigh);
    // new JoystickButton(operatorJoystick, Button.kLeftBumper.value).whileTrue(elevatorToLow);

    new JoystickButton(operatorJoystick, Button.kRightBumper.value).onTrue(scorePieceLow);
    new JoystickButton(operatorJoystick, Button.kLeftBumper.value).onTrue(scorePieceHigh);

    new JoystickButton(operatorJoystick, Button.kB.value).onTrue(intakePieceGround);
    new JoystickButton(operatorJoystick, Button.kY.value).onTrue(intakePieceSubstation);

    new JoystickButton(operatorJoystick, Button.kA.value).onTrue(smartRetract);
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
    // driveBaseSubsystem.setDefaultCommand(testing);
    armSubsystem.setDefaultCommand(moveArmWithJoystickAnalog);
    // wristSubsystem.setDefaultCommand(moveWristWithJoystick);
    elevatorSubsystem.setDefaultCommand(moveElevatorWithJoystickAnalog);
    // gripperSubsystem.setDefaultCommand(runGripperWithJoystick);
  }
}
