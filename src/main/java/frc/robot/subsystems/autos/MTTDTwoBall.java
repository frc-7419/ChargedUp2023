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

public class MTTDTwoBall extends ParallelCommandGroup {

    public MTTDTwoBall(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem,
            ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, FeederSubsystem feederSubsystem,
            LoaderSubsystem loaderSubsystem, LEDSubsystem ledSubsystem, TurretSubsystem turretSubsystem,
            IntakeSolenoidSubsystem intakeSolenoidSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                sequence(

                        parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                                new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem),
                                new RunFeeder(feederSubsystem, 0.9),
                                new RunLoader(loaderSubsystem, 1)).withTimeout(1.2),
                        parallel(
                                new InstantCommand(intakeSolenoidSubsystem::actuateSolenoid),
                                new RunIntake(intakeSubsystem, 1), 
                                new RunLoader(loaderSubsystem, 0.5)
                        ).withTimeout(2),
                        parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                                new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem),
                                new RunFeeder(feederSubsystem, 0.9),
                                new RunLoader(loaderSubsystem, 1)).withTimeout(1),
                        new StraightWithMotionMagic(driveBaseSubsystem, -40),
                        new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, 2, PIDConstants.GyrokP180,
                                PIDConstants.GyrokI180, PIDConstants.GyrokD180),
                        new InstantCommand(driveBaseSubsystem::coast, driveBaseSubsystem)));

    }

}
