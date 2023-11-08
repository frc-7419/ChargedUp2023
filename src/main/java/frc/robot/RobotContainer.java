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
import frc.robot.commands.actions.AutoScorePiece;
import frc.robot.commands.actions.AutoScorePieceRetract;
import frc.robot.commands.actions.AutoIntakePiece;
import frc.robot.commands.actions.IntakePieceDouble;
import frc.robot.commands.actions.ScorePiece;
import frc.robot.commands.actions.SmartRetract;
import frc.robot.commands.actions.ZeroSensors;
import frc.robot.commands.autos.AutoHigh;
import frc.robot.commands.autos.AutoTwoPieceHigh;
import frc.robot.commands.autos.AutoHighBalance;
import frc.robot.commands.autos.AutoHighStop;
import frc.robot.commands.autos.Balance;
import frc.robot.commands.autos.Mobility;
import frc.robot.commands.autos.MobilityBalance;
import frc.robot.commands.paths.TurnToAngleFieldRelative;
import frc.robot.commands.paths.TurnToAngleRobotRelative;
import frc.robot.constants.NodeConstants;
import frc.robot.constants.GripperConstants.GripperState;
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
import frc.robot.subsystems.gripper.AutoRunGripper;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.SmartBalance;
import frc.robot.subsystems.gyro.TurnWithGyro;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.led.RunLed;
import frc.robot.subsystems.state.StateMachine;
import frc.robot.subsystems.state.SwitchState;
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
        private final StateMachine stateMachine = new StateMachine();
        private final LedSubsystem ledSubsystem = new LedSubsystem();

        // // Commands
        private final SwitchState switchState = new SwitchState(stateMachine, operatorJoystick);
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
        private final ArmToSetpointWithFeedforward armToTestSetpoint = new ArmToSetpointWithFeedforward(armSubsystem,
                        NodeState.SUBSTATION);
        private final IntakePieceDouble intakePieceDoubleSub = new IntakePieceDouble(elevatorSubsystem, armSubsystem,
                        wristSubsystem, gripperSubsystem, stateMachine, NodeState.SUBSTATION);
        private final ElevatorToSetpointWithFeedForward elevatorToLow = new ElevatorToSetpointWithFeedForward(
                        elevatorSubsystem,
                        NodeConstants.NodeState.LOW);

        private final AutoRunGripper AutoRunGripper = new AutoRunGripper(gripperSubsystem, stateMachine);
        private final ElevatorToSetpointWithFeedForward elevatorToHigh = new ElevatorToSetpointWithFeedForward(
                        elevatorSubsystem,
                        NodeConstants.NodeState.HIGH);
        private final MoveWristWithJoystick moveWristWithJoystick = new MoveWristWithJoystick(wristSubsystem,
                        driverJoystick);
        private final RunGripperWithJoystick runGripperWithJoystick = new RunGripperWithJoystick(gripperSubsystem,
                        operatorJoystick, ledSubsystem);
        private final ScorePiece scorePieceHigh = new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem,
                        NodeState.HIGH);
        private final AutoScorePieceRetract autoScorePieceHighCube = new AutoScorePieceRetract(elevatorSubsystem,
                        armSubsystem, wristSubsystem, gripperSubsystem, NodeState.LOW, GripperState.SCORE_CUBE, stateMachine);
        private final ScorePiece scorePieceMid = new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem,
                        NodeState.LOW);

        private final AutoIntakePiece intakePieceSingleSub = new AutoIntakePiece(elevatorSubsystem, stateMachine, armSubsystem,
                        gripperSubsystem, wristSubsystem);

        private final SmartRetract smartRetract = new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem);

        private final RunLed runLed = new RunLed(ledSubsystem, operatorJoystick);

        // // Autonomous
        private SendableChooser<Command> autonomousChooser = new SendableChooser<>();
        private final TurnWithGyro turnWithGyro180 = new TurnWithGyro(driveBaseSubsystem, gyroSubsystem, 180, 0.5);
        // Path Planning Commands

        // TODO will use when testing path planning
        // private final MoveToMid moveToPortal = new MoveToMid(driveBaseSubsystem);
        private final TurnToAngleRobotRelative turn180RobotRelative = new TurnToAngleRobotRelative(driveBaseSubsystem,
                        180);
        private final TurnToAngleFieldRelative turn180FieldRelative = new TurnToAngleFieldRelative(driveBaseSubsystem,
                        180);
        private final MobilityBalance mobilityBalance = new MobilityBalance(driveBaseSubsystem, gyroSubsystem);

        public RobotContainer() {
                configureButtonBindings();
                configureAutoSelector();
        }

        private void configureButtonBindings() {
                new JoystickButton(operatorJoystick, Button.kX.value).onTrue(scorePieceHigh);
                new JoystickButton(operatorJoystick, Button.kA.value).onTrue(intakePieceSingleSub);
                new JoystickButton(operatorJoystick, Button.kY.value).onTrue(scorePieceMid);
                new JoystickButton(operatorJoystick, Button.kB.value).onTrue(smartRetract);
                new JoystickButton(operatorJoystick, Button.kBack.value).onTrue(autoScorePieceHighCube);
                new JoystickButton(operatorJoystick, Button.kStart.value).onTrue(intakePieceDoubleSub);
                new JoystickButton(driverJoystick, Button.kY.value).whileTrue(smartBalance);
                new JoystickButton(driverJoystick, XboxController.Button.kStart.value)
                                .onTrue(new ZeroSensors(elevatorSubsystem, armSubsystem, wristSubsystem));
                new JoystickButton(operatorJoystick, XboxController.Button.kRightStick.value)
                                .onTrue(new ZeroSensors(elevatorSubsystem, armSubsystem, wristSubsystem));
                new JoystickButton(driverJoystick, Button.kRightStick.value)
                                .onTrue(new InstantCommand(gyroSubsystem::zeroYaw));
        }

        private void configureAutoSelector() {
                autonomousChooser.setDefaultOption("Auto High",
                                new AutoHigh(driveBaseSubsystem, elevatorSubsystem, armSubsystem,
                                                wristSubsystem, gripperSubsystem, gyroSubsystem, NodeState.HIGH, stateMachine));
                autonomousChooser.addOption("Auto High + Balance",
                                new AutoHighBalance(driveBaseSubsystem, elevatorSubsystem,
                                                armSubsystem, wristSubsystem, gripperSubsystem, gyroSubsystem,
                                                NodeState.HIGH, stateMachine));
                autonomousChooser.addOption("Auto Two Piece High",
                                new AutoTwoPieceHigh(driveBaseSubsystem, elevatorSubsystem,
                                                armSubsystem, wristSubsystem, gripperSubsystem, gyroSubsystem, stateMachine));
                autonomousChooser.addOption("Hybrid", new Mobility(driveBaseSubsystem));
                autonomousChooser.addOption("Hybrid + Balance", new MobilityBalance(driveBaseSubsystem, gyroSubsystem));
                autonomousChooser.addOption("Auto High Stop",
                                new AutoHighStop(driveBaseSubsystem, elevatorSubsystem, armSubsystem,
                                                wristSubsystem, gripperSubsystem, gyroSubsystem, NodeState.HIGH, stateMachine));
                SmartDashboard.putData(autonomousChooser);
        }

        public Command getAutonomousCommand() {
                // ledSubsystem.rainbowLED(0);
                // return autonomousChooser.getSelected();
                return new AutoTwoPieceHigh(driveBaseSubsystem, elevatorSubsystem, armSubsystem, wristSubsystem,
                                gripperSubsystem,
                                gyroSubsystem, stateMachine);
                // return balance;
                // return new Mobility(driveBaseSubsystem);
                // return mobility;xxxxxxxxxxxxxxxxx
                // return new MobilityBalance(driveBaseSubsystem, gyroSubsystem);
                // return new AutoHigh(driveBaseSubsystem, elevatorSubsystem, armSubsystem,
                // wristSubsystem, gripperSubsystem, gyroSubsystem, NodeState.HIGH);
                // return new Balance(driveBaseSubsystem, gyroSubsystem);
                // return new WaitCommand(5);
        }

        public void setDefaultCommands() {
                driveBaseSubsystem.setDefaultCommand(arcadeDrive);
                gripperSubsystem.setDefaultCommand(runGripperWithJoystick);
                wristSubsystem.setDefaultCommand(moveWristWithJoystick);
                armSubsystem.setDefaultCommand(moveArmWithJoystickAnalog);
                elevatorSubsystem.setDefaultCommand(moveElevatorWithJoystickAnalog);
                ledSubsystem.setDefaultCommand(runLed);
                stateMachine.setDefaultCommand(switchState);
        }

        public void zeroSensor(String allianceColor, String allianceSide) {

                gyroSubsystem.zeroPitch();
                gyroSubsystem.zeroRoll();
                gyroSubsystem.zeroYaw(allianceColor);
                elevatorSubsystem.zeroEncoder();
                armSubsystem.zeroEncoder();
                wristSubsystem.zeroEncoder();
        }

        public void zeroSensor() {
                gyroSubsystem.zeroPitch();
                gyroSubsystem.zeroRoll();
                elevatorSubsystem.zeroEncoder();
                armSubsystem.zeroEncoder();
                wristSubsystem.zeroEncoder();
        }
}
