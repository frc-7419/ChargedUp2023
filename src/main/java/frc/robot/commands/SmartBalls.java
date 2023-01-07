// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class SmartBalls extends CommandBase {
  private XboxController joystick1;
  private XboxController joystick2;
  private IntakeSubsystem intakeSubsystem;
  private LoaderSubsystem loaderSubsystem;
  private FeederSubsystem feederSubsystem;
  private BeamBreakSubsystem beamBreakSubsystem;
  private boolean isBalling = false;
  public SmartBalls(XboxController joystick1, XboxController joystick2, IntakeSubsystem intakeSubsystem,
      LoaderSubsystem loaderSubsystem, FeederSubsystem feederSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
    this.joystick1 = joystick1;
    this.joystick2 = joystick2;
    this.intakeSubsystem = intakeSubsystem;
    this.loaderSubsystem = loaderSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.beamBreakSubsystem = beamBreakSubsystem;
    addRequirements(intakeSubsystem, loaderSubsystem, feederSubsystem, beamBreakSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (joystick2.getRightBumper()) {
      isBalling = true;
      if (!beamBreakSubsystem.getBeamBreakActivated()) {
        SmartDashboard.putBoolean("SmartLoad Finished", false);
        feederSubsystem.setVoltage(-Constants.PowerConstants.FeederVoltage);
        loaderSubsystem.setPower(-0.6);
      } else {
        feederSubsystem.setVoltage(-0.3*Constants.PowerConstants.FeederVoltage);
        loaderSubsystem.setPower(0.3);
        SmartDashboard.putBoolean("SmartLoad Finished", true);
      }
    } else {
      isBalling = false;
      if (joystick2.getRightTriggerAxis() > 0.05) {
        intakeSubsystem.setPower(1);
        loaderSubsystem.setPower(0.6);
      } else if (joystick2.getLeftTriggerAxis() > 0.05) {
        intakeSubsystem.setPower(-1);
        loaderSubsystem.setPower(-0.4);
      } else {
        intakeSubsystem.setPower(0);
        loaderSubsystem.setPower(0);
      }
    }
    if (joystick1.getLeftBumper()) {
      feederSubsystem.setVoltage(-Constants.PowerConstants.FeederVoltage);
    } else if (joystick1.getRightBumper()) {
      feederSubsystem.setVoltage(Constants.PowerConstants.FeederVoltage);
    } else if(!isBalling){
      feederSubsystem.setVoltage(0);
    }
  }

  // if (beamBreakSubsystem.getBeamBreakActivated()) {
  // SmartDashboard.putBoolean("SmartLoad Finished", false);
  // feederSubsystem.setPower(-0.5);
  // loaderSubsystem.setPower(-0.5);
  // } else {
  // feederSubsystem.setPower(0);
  // loaderSubsystem.setPower(0);
  // ledSubsystem.setLEDColor(0, 0, 255);
  // SmartDashboard.putBoolean("SmartLoad Finished", true);
  // }
  

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
