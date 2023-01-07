//period 6 file
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

public class TwoBallAutoInterpolation extends ParallelCommandGroup {

    public TwoBallAutoInterpolation(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, LoaderSubsystem loaderSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, LEDSubsystem ledSubsystem, IntakeSolenoidSubsystem intakeSolenoidSubsystem) {
        addCommands(
            sequence(
                parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocity(shooterSubsystem, 37, 30))
                    .withTimeout(0.75), // gttv while aligning turret

                // shoot preload
                parallel(
                    new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                    new GetToTargetVelocity(shooterSubsystem, 37, 30),
                    new RunFeeder(feederSubsystem, 1),
                    new RunLoader(loaderSubsystem, 1)
                ).withTimeout(1.5), // tune time
                
                new InstantCommand(intakeSolenoidSubsystem::retractSolenoid, intakeSolenoidSubsystem),

                // turn 180 while braking turret
                new BrakeTurret(turretSubsystem)
                    .deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 175, 3, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180))
                    .withTimeout(1.5),

                new WaitCommand(0.25),  
                
                // deploy intake
                new InstantCommand(intakeSolenoidSubsystem::actuateSolenoid, intakeSolenoidSubsystem),

                // move forward and running intake + loader
                parallel(
                    new RunIntake(intakeSubsystem, 1),
                    new RunLoader(loaderSubsystem, 0.6)
                ).deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, 46))
                .withTimeout(2.5),
                
                new WaitCommand(0.25),

                // retract intake
                new InstantCommand(intakeSolenoidSubsystem::retractSolenoid, intakeSolenoidSubsystem),

                // turn 180 while braking turret
               // turn 185 while braking turret
               new BrakeTurret(turretSubsystem)
                    .deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 185, 1.5, PIDConstants.GyrokP185, PIDConstants.GyrokI185, PIDConstants.GyrokD185)).withTimeout(2),
                

                // exact shooter velocities:
                // parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocity(shooterSubsystem, 43, 40))
                //     .withInterrupt(() -> shooterSubsystem.bothOnTarget())
                //     .withTimeout(1.15), // gttv while aligning turret

                // gttv using limelight
                parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem))
                    .withInterrupt(() -> shooterSubsystem.bothOnTarget())
                    .withTimeout(1.15), // gttv while aligning turret
                
                // // shoot second ball, exact velocities:
                // parallel(
                //     new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                //     new GetToTargetVelocity(shooterSubsystem, 45.5, 43), // mainting the specific velocity
                //     new RunLoader(loaderSubsystem, 1),
                //     new RunFeeder(feederSubsystem, 0.5)
                // ).withTimeout(1.5),

                // shoot second ball, using limelight velocity
                parallel(
                    new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                    new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem), // mainting the specific velocity
                    new RunLoader(loaderSubsystem, 1),
                    new RunFeeder(feederSubsystem, 0.5)
                ).withTimeout(1.5),

                new InstantCommand(driveBaseSubsystem::coast, driveBaseSubsystem)
            )
        );
        addCommands(new SetLEDColor(ledSubsystem, limelightSubsystem, driveBaseSubsystem));
    }
}