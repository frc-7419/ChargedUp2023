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
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.SetLEDColor;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.GetToTargetVelocityArbitraryFeedforward;
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.BrakeTurret;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.RunIntake;

public class CCCTwoBallCopy extends ParallelCommandGroup {

    public CCCTwoBallCopy(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem,
            ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, FeederSubsystem feederSubsystem,
            LoaderSubsystem loaderSubsystem, LEDSubsystem ledSubsystem, TurretSubsystem turretSubsystem,
            IntakeSolenoidSubsystem intakeSolenoidSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                sequence(

                        // new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 9850, 6150,
                        // 0.0485, 0.0495).withTimeout(2),
                        new StraightWithMotionMagic(driveBaseSubsystem, -40),
                        parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                        new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 8100, 10200, 0.04874, 0.049)
                                ).withTimeout(2), // gttv while aligning turret

                        // parallel(new RunLoader(loaderSubsystem, 1.0), new
                        // GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 9850, 6150, 0.0485,
                        // 0.0495)).withTimeout(2.5),

                        // shoot preload
                        // path planner
                        parallel(
                                
                                new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                                new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 8050, 9500, 0.04874, 0.049),
                                new RunFeeder(feederSubsystem, 0.9),
                                new RunLoader(loaderSubsystem, 1)).withTimeout(0.25),

                        parallel(
                                new RunFeeder(feederSubsystem, -0.9),
                                new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 8000, 9900, 0.04874, 0.049)
                        ).withTimeout(2),

                        parallel(
                        
                                new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                                new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 7900, 9700, 0.04874, 0.049),
                                new StraightWithMotionMagic(driveBaseSubsystem, 6),
                                new RunFeeder(feederSubsystem, 0.9),
                                new RunLoader(loaderSubsystem, 1)).withTimeout(1),
                                // tune time
                        // new StraightWithMotionMagic(driveBaseSubsystem, 5),
                        // parallel(
                        //         new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                        //         new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 7900, 9900, 0.04874, 0.049),
                        //         // new InstantCommand(intakeSolenoidSubsystem::actuateSolenoid,
                        //         //         intakeSolenoidSubsystem),
                        //         new RunLoader(loaderSubsystem, 0.5),
                        //         new RunIntake(intakeSubsystem, 1)

                        // ).withTimeout(3),

                        // parallel(
                        //         new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                        //         new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 7900, 9900, 0.04874, 0.049),
                        //         new RunFeeder(feederSubsystem, 1),
                        //         new RunLoader(loaderSubsystem, 1)).withTimeout(3), // tune time

                        // parallel(
                        //         new InstantCommand(intakeSolenoidSubsystem::retractSolenoid,
                        //                 intakeSolenoidSubsystem)).withTimeout(2),
        new StraightWithMotionMagic(driveBaseSubsystem, 40),
            new InstantCommand(driveBaseSubsystem::coast, driveBaseSubsystem)
        // new StraightWithMotionMagic(driveBaseSubsystem, -80.88)
        ));

    }

}
