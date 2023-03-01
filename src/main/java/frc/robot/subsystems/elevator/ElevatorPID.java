// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorPID extends PIDCommand {
  /** Creates a new ElevatorPID. */
  public ElevatorPID(ElevatorSubsystem elevatorSubsystem, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(0.001, 0, 0),
        // This should return the measurement
        elevatorSubsystem::getElevatorPosition,
        // This should return the setpoint (can also be a constant)
        setpoint,
        // This uses the output
        output -> {
          // Use the output here
          elevatorSubsystem.setPower(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
    // Configure additional PID options by calling `getController` here.b
    getController().setTolerance(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
