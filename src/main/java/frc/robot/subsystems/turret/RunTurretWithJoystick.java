// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class RunTurretWithJoystick extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private XboxController joystick;
  private PIDController pidController;
  private double kSpeed;
  private double pidOutput;

  private double tx, tv;

  public RunTurretWithJoystick(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, XboxController joystick, double kSpeed) {
    this.turretSubsystem = turretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.joystick = joystick;
    this.kSpeed = kSpeed;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.coast();

    // pidController = new PIDController(PIDConstants.TurretKp, 0, 0);
    // pidController.setSetpoint(0);
    // pidController.setTolerance(1);
  }

  @Override
  public void execute() {
    tx = limelightSubsystem.getTx();
    tv = limelightSubsystem.getTv();

    // if (joystick.getLeftX() != 0) {
    //   turretSubsystem.coast();
    //   turretSubsystem.setPower(kSpeed*-joystick.getLeftX());
    // }
    // else if (joystick.getRightX() != 0) {
    //   turretSubsystem.coast();
    //   turretSubsystem.setPower(kSpeed*joystick.getRightX());
    // }
    if (joystick.getLeftTriggerAxis() != 0) {
      turretSubsystem.coast();
      turretSubsystem.setPower(kSpeed*joystick.getLeftTriggerAxis());
    }
    else if (joystick.getRightTriggerAxis() != 0) {
      turretSubsystem.coast();
      turretSubsystem.setPower(kSpeed*-joystick.getRightTriggerAxis());
    }    
    // else if (tv == 1.0) {
    //   pidController = new PIDController(PIDConstants.TurretKp, 0, 0);
    //   pidOutput = pidController.calculate(tx);
    //   turretSubsystem.setPower(pidOutput);
    // }
    else {
      turretSubsystem.setPower(0);
      turretSubsystem.brake();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
