// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;

import static frc.robot.Constants.PIDConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartExtendedArm extends PIDCommand {
  public SmartExtendedArm(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    // super(
    //     // The controller that the command will use
    //    ,

        // This should return the measurement
        armSubsystem::getExtendedAngle,

    //     // This should return the setpoint (can also be a constant)
    //     setpoint,

    //     // This uses the output
    //     output -> {
    //       // Use the output here
    //       armSubsystem.setExtendedPower(output);
    //     });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
    
    // Configure additional PID options by calling `getController` here.
    // getController().setTolerance(ExtendedArmKTolerance);
    // getController().enableContinuousInput(0, 0);
  }

  @Override
  public void execute() {
    double output = controller.calculate(armSubsystem.getExtendedAngle());
    armSubsystem.setExtendedPower(output);
    SmartDashboard.putNumber("angle", armSubsystem.getExtendedAngle());
    SmartDashboard.putNumber("output", output);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
