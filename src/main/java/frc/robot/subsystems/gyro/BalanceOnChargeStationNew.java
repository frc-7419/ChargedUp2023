// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class BalanceOnChargeStationNew extends CommandBase {
  /** Creates a new SmartBalanceNew. */
  private DriveBaseSubsystem driveBaseSubsystem;

  private GyroSubsystem gyroSubsystem;
  private PIDController yawAngleController;
  private PIDController pitchAngleController;
  private PIDController speedController;

  public BalanceOnChargeStationNew(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    addRequirements(driveBaseSubsystem, gyroSubsystem);
  }

  /** Initialize the angle and speed PID controllers */
  @Override
  public void initialize() {
    yawAngleController =
        new PIDController(
            PIDConstants.BalanceAngleKp, PIDConstants.BalanceAngleKi, PIDConstants.BalanceAngleKd);
    pitchAngleController =
        new PIDController(
            PIDConstants.BalanceAngleKp, PIDConstants.BalanceAngleKi, PIDConstants.BalanceAngleKd);
    speedController =
        new PIDController(
            PIDConstants.BalanceSpeedKp, PIDConstants.BalanceSpeedKi, PIDConstants.BalanceSpeedKd);
    yawAngleController.setSetpoint(0);
    pitchAngleController.setSetpoint(0);
    speedController.setSetpoint(PIDConstants.BalanceSpeed);
    yawAngleController.setTolerance(PIDConstants.BalanceAngleKTolerance);
    pitchAngleController.setTolerance(PIDConstants.BalanceAngleKTolerance);
    speedController.setTolerance(PIDConstants.BalanceSpeedKTolerance);
  }

  /**
   * Calculate the PID output based on the encoder velocity reading and the gyroscope pitch
   * measurements. Set the drivetrain power based on PID output
   */
  @Override
  public void execute() {
    double pitch = gyroSubsystem.getRoll();
    double calculatedDirection = pitchAngleController.calculate(pitch);

    double yaw = gyroSubsystem.getYaw();
    // TODO: will use during testing
    double calculatedYaw = yawAngleController.calculate(yaw);

    double speed = driveBaseSubsystem.getLeftVelocityInMeters();
    double calculatedPower = speedController.calculate(speed);

    double calculatedOutput = Math.copySign(calculatedPower, calculatedDirection);

    // Setting the power
    double leftPower =
        -calculatedOutput - Math.copySign(PIDConstants.BalanceSpeedkF, calculatedDirection);
    double rightPower =
        -calculatedOutput - Math.copySign(PIDConstants.BalanceSpeedkF, calculatedDirection);

    driveBaseSubsystem.setLeftPower(leftPower);
    driveBaseSubsystem.setRightPower(rightPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setAllPower(0);
    driveBaseSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pitchAngleController.atSetpoint();
  }
}
