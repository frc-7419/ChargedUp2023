// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetToTarget extends ParallelCommandGroup {
  /** Creates a new GetToTarget. */
  public GetToTarget(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StraightWithMotionMagic(driveBaseSubsystem, Units.metersToInches(1))
      //  new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -driveBaseSubsystem.getAngle(), 2, 0.0001, 0, 0)
    );
  }
}
