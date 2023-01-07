// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeeder;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.SetLEDColor;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.BrakeTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ThreeBallAutoInterpolation extends ParallelCommandGroup {

  public ThreeBallAutoInterpolation(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, LoaderSubsystem loaderSubsystem, FeederSubsystem feederSubsystem, DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, IntakeSubsystem intakeSubsystem, IntakeSolenoidSubsystem intakeSolenoidSubsystem, LEDSubsystem ledSubsystem) {
    addCommands(
      sequence(
        // gttv and align turret
        parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocity(shooterSubsystem, 37, 30))
          .withTimeout(0.75), // gttv while aligning turret

        // shoot preload
        parallel(
            new AlignTurretDefault(turretSubsystem, limelightSubsystem),
            new GetToTargetVelocity(shooterSubsystem, 37, 30),
            new RunFeeder(feederSubsystem, 1),
            new RunLoader(loaderSubsystem, 1)
        ).withTimeout(1.5), // tune time

        // retract intake
        new InstantCommand(intakeSolenoidSubsystem::retractSolenoid, intakeSolenoidSubsystem),

        // turn 180 while braking turret
        new BrakeTurret(turretSubsystem)
            .deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, 2, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180))
            .withTimeout(1.25),

        new WaitCommand(0.25),  
              
        // deploy intake
        new InstantCommand(intakeSolenoidSubsystem::actuateSolenoid, intakeSolenoidSubsystem),

        // move forward while running intake + loader
        parallel(
            new RunIntake(intakeSubsystem, 1),
            new RunLoader(loaderSubsystem, 0.6)
        ).deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, 46))
        .withTimeout(2),
        
        new WaitCommand(0.25),

        // retract intake
        // new InstantCommand(intakeSolenoidSubsystem::retractSolenoid, intakeSolenoidSubsystem),

        // turn 117 while braking turret
        new BrakeTurret(turretSubsystem)
            .deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 111.5, 2, PIDConstants.GyrokP115, PIDConstants.GyrokI115, PIDConstants.GyrokD115))
            .withTimeout(1.25),

        new WaitCommand(0.15),

        // deploy intake
        // new InstantCommand(intakeSolenoidSubsystem::actuateSolenoid, intakeSolenoidSubsystem),

        // move forward 132.5 in
        parallel(
            new RunIntake(intakeSubsystem, 1),
            new RunLoader(loaderSubsystem, 0.6))
        .deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, 132.5))
        .withTimeout(2.5),

        new WaitCommand(0.3),

        // new AlignTurretDefault(turretSubsystem, limelightSubsystem)
        //   .deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, -60))
        //   .withTimeout(1),

        // turn 110 while braking turret
        new BrakeTurret(turretSubsystem)
        .deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 110, 2, PIDConstants.GyrokP115, PIDConstants.GyrokI115, PIDConstants.GyrokD115))
        .withTimeout(1.15),
        
        // gttv and align, exact velocity
        // parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocity(shooterSubsystem, 37, 30))
        //   .withTimeout(0.5),

        // gttv with limelight and align
        parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem))
          .withTimeout(0.5), // gttv while aligning turret
        
        // shoot both balls with exact velocity
        // parallel(
        //     new AlignTurretDefault(turretSubsystem, limelightSubsystem),
        //     new GetToTargetVelocity(shooterSubsystem, 43, 36),
        //     new RunFeeder(feederSubsystem, 1),
        //     new RunLoader(loaderSubsystem, 1)
        // ).withTimeout(2.15), // tune time

        // shoot both balls with limelight velocity
        parallel(
            new AlignTurretDefault(turretSubsystem, limelightSubsystem),
            new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem),
            new RunFeeder(feederSubsystem, 1),
            new RunLoader(loaderSubsystem, 1)
        ).withTimeout(2.15), // tune time
        
        new InstantCommand(driveBaseSubsystem::coast, driveBaseSubsystem)
      ));
      
      addCommands(new SetLEDColor(ledSubsystem, limelightSubsystem, driveBaseSubsystem));
  }
}

// shoot preload
// turn 180
// forward to ball while intake (parallel)
// turn toward third ball
// forward to third ball while intake (parallel)
// turn toward hub
// shoot both
