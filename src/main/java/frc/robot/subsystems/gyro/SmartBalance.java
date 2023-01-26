// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import static frc.robot.Constants.PIDConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartBalance extends PIDCommand {
  /** Creates a new SmartBalance. */
  public SmartBalance(GyroSubsystem gyroSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(BalanceKp, BalanceKi, BalanceKd),

        // This should return the measurement
        gyroSubsystem::getYaw,

        // This should return the setpoint (can also be a constant)
        0,

        // This uses the output
        output -> {
          // Use the output here
          driveBaseSubsystem.setLeftPower(-output);
          driveBaseSubsystem.setRightPower(output);
        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gyroSubsystem, driveBaseSubsystem);
    
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(BalanceKTolerance);
    getController().enableContinuousInput(-180, 180);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
