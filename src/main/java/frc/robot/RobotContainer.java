package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.actions.AutoIntakePiece;
import frc.robot.commands.actions.AutoScorePieceRetract;
import frc.robot.commands.actions.IntakePieceDouble;
import frc.robot.commands.actions.ScorePiece;
import frc.robot.commands.actions.SmartRetract;
import frc.robot.commands.actions.ZeroSensors;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.constants.NodeConstants;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToSetpointWithFeedforward;
import frc.robot.subsystems.arm.ArmWithMotionMagic;
import frc.robot.subsystems.arm.MoveArmWithJoystickAnalog;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpointWithFeedForward;
import frc.robot.subsystems.elevator.MoveElevatorWithJoystickAnalog;
import frc.robot.subsystems.gripper.AutoRunGripper;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripperWithJoystick;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.SmartBalance;
import frc.robot.subsystems.led.LedSubsystem;
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

        // Commands
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
        private final AutoScorePieceRetract autoScorePieceLow = new AutoScorePieceRetract(elevatorSubsystem,
                        armSubsystem, wristSubsystem, gripperSubsystem, NodeState.LOW, GripperState.SCORE_CUBE,
                        stateMachine);
        private final AutoScorePieceRetract autoScorePieceHigh = new AutoScorePieceRetract(elevatorSubsystem,
                        armSubsystem, wristSubsystem, gripperSubsystem, NodeState.HIGH, GripperState.SCORE_CONE,
                        stateMachine);
        private final ScorePiece scorePieceMid = new ScorePiece(elevatorSubsystem, armSubsystem, wristSubsystem,
                        NodeState.LOW);

        private final ArmWithMotionMagic testArmWithMotionMagic = new ArmWithMotionMagic(armSubsystem,
                        NodeState.HIGH.armSetpoint);

        private final AutoIntakePiece intakePieceSingleSub = new AutoIntakePiece(elevatorSubsystem, stateMachine,
                        armSubsystem,
                        gripperSubsystem, wristSubsystem);

        private final SmartRetract smartRetract = new SmartRetract(elevatorSubsystem, armSubsystem, wristSubsystem);

        public RobotContainer() {
                configureButtonBindings();
        }

        private void configureButtonBindings() {
                new JoystickButton(driverJoystick, Button.kB.value)
                                .onTrue(new InstantCommand(driveBaseSubsystem::brake));
                new JoystickButton(driverJoystick, Button.kA.value)
                                .onTrue(new InstantCommand(driveBaseSubsystem::coast));
                new JoystickButton(operatorJoystick, Button.kX.value).onTrue(scorePieceHigh);
                new JoystickButton(operatorJoystick, Button.kA.value).onTrue(intakePieceSingleSub);
                new JoystickButton(operatorJoystick, Button.kY.value).onTrue(scorePieceMid);
                new JoystickButton(operatorJoystick, Button.kB.value).onTrue(smartRetract);
                new JoystickButton(operatorJoystick, Button.kBack.value).onTrue(autoScorePieceLow);
                new JoystickButton(operatorJoystick, Button.kStart.value).onTrue(autoScorePieceHigh);
                new JoystickButton(driverJoystick, Button.kY.value).whileTrue(smartBalance);
                new JoystickButton(driverJoystick, XboxController.Button.kStart.value)
                                .onTrue(new ZeroSensors(elevatorSubsystem, armSubsystem, wristSubsystem));
                new JoystickButton(operatorJoystick, XboxController.Button.kRightStick.value)
                                .onTrue(new ZeroSensors(elevatorSubsystem, armSubsystem, wristSubsystem));
                new JoystickButton(driverJoystick, Button.kRightStick.value)
                                .onTrue(new InstantCommand(gyroSubsystem::zeroYaw));
        }

        public Command getAutonomousCommand() {
                return new WaitCommand(0);
        }

        public void setDefaultCommands() {
                driveBaseSubsystem.setDefaultCommand(arcadeDrive);
                gripperSubsystem.setDefaultCommand(runGripperWithJoystick);
                wristSubsystem.setDefaultCommand(moveWristWithJoystick);
                armSubsystem.setDefaultCommand(moveArmWithJoystickAnalog);
                elevatorSubsystem.setDefaultCommand(moveElevatorWithJoystickAnalog);
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
