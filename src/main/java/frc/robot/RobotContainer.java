package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToSetpoint;
import frc.robot.subsystems.arm.HomeArm;
import frc.robot.subsystems.arm.MoveArmWithJoystickAnalog;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.Testing;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpoint;
import frc.robot.subsystems.elevator.MoveElevatorWithJoystickAnalog;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripperWithJoystick;
import frc.robot.subsystems.gyro.BalanceOnChargeStationNew;
import frc.robot.subsystems.gyro.GyroSubsystem;
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
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final GripperSubsystem gripperSubsystem = new GripperSubsystem();

  // Commands
  //   private final ArcadeDrive arcadeDrive = new ArcadeDrive(driverJoystick, driveBaseSubsystem);
  private final Testing testing = new Testing(driveBaseSubsystem, driverJoystick);

  private final BalanceOnChargeStationNew smartBalanceNew =
      new BalanceOnChargeStationNew(driveBaseSubsystem, gyroSubsystem);

  private final ArmToSetpoint armToIntakeSetpoint =
      new ArmToSetpoint(armSubsystem, ArmConstants.intakeSetpoint);

  private final ArmToSetpoint armToScoreSetpoint =
      new ArmToSetpoint(armSubsystem, ArmConstants.scoreSetpoint);

  private final ElevatorToSetpoint elevatorToGround =
      new ElevatorToSetpoint(elevatorSubsystem, NodeState.GROUND);
  private final ElevatorToSetpoint elevatorToSubstation =
      new ElevatorToSetpoint(elevatorSubsystem, NodeState.SUBSTATION);
  private final ElevatorToSetpoint elevatorToLow =
      new ElevatorToSetpoint(elevatorSubsystem, NodeState.LOW);
  private final ElevatorToSetpoint elevatorToHigh =
      new ElevatorToSetpoint(elevatorSubsystem, NodeState.HIGH);

  // private final MoveElevatorWithJoystick moveElevatorWithJoystick = new
  // MoveElevatorWithJoystick(elevatorSubsystem, driverJoystick);
  private final HomeArm homeArm = new HomeArm(armSubsystem);
  // private final MoveArmWithJoystick moveArmWithJoystick =
  // new MoveArmWithJoystick(armSubsystem, driverJoystick);
  private final MoveWristWithJoystick moveWristWithJoystick =
      new MoveWristWithJoystick(wristSubsystem, driverJoystick);
  // private final SmartWrist smartWrist = new SmartWrist(wristSubsystem, 10000);
  private final RunGripperWithJoystick runGripperWithJoystick =
      new RunGripperWithJoystick(gripperSubsystem, driverJoystick);
  private final MoveElevatorWithJoystickAnalog moveElevatorWithJoystickAnalog =
      new MoveElevatorWithJoystickAnalog(elevatorSubsystem, operatorJoystick);
  private final MoveArmWithJoystickAnalog moveArmWithJoystickAnalog =
      new MoveArmWithJoystickAnalog(armSubsystem, operatorJoystick);

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
    new JoystickButton(driverJoystick, Button.kA.value).whileTrue(armToIntakeSetpoint);
    new JoystickButton(driverJoystick, Button.kB.value).whileTrue(armToScoreSetpoint);
    new JoystickButton(driverJoystick, Button.kY.value).whileTrue(homeArm);
    // new JoystickButton(driverJoystick, Button.kLeftBumper.value).whileTrue(smartWrist);
    // new JoystickButton(driverJoystick, Button.kRightBumper.value).whileTrue(elevatorToSetpoint);
    new JoystickButton(operatorJoystick, Button.kX.value).whileTrue(elevatorToGround);
    new JoystickButton(operatorJoystick, Button.kRightBumper.value).whileTrue(elevatorToHigh);
    new JoystickButton(operatorJoystick, Button.kLeftBumper.value).whileTrue(elevatorToLow);
    new JoystickButton(operatorJoystick, Button.kY.value).whileTrue(elevatorToSubstation);
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
    // driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    // driveBaseSubsystem.setDefaultCommand(testing);
    armSubsystem.setDefaultCommand(moveArmWithJoystickAnalog);
    wristSubsystem.setDefaultCommand(moveWristWithJoystick);
    elevatorSubsystem.setDefaultCommand(moveElevatorWithJoystickAnalog);
    gripperSubsystem.setDefaultCommand(runGripperWithJoystick);
  }
}
