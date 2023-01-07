package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeeder;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.SetLEDColor;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.GetToTargetVelocityArbitraryFeedforward;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.TurretSubsystem;

public class OneBallAuto extends ParallelCommandGroup {

    public OneBallAuto(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, FeederSubsystem feederSubsystem, LoaderSubsystem loaderSubsystem, LEDSubsystem ledSubsystem, TurretSubsystem turretSubsystem) {
        addCommands(
            sequence(
                // new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 9850, 6150, 0.0485, 0.0495).withTimeout(2),
                parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocity(shooterSubsystem, 37, 30))
                    .withTimeout(0.75), // gttv while aligning turret
                    
                // parallel(new RunLoader(loaderSubsystem, 1.0), new GetToTargetVelocityArbitraryFeedforward(shooterSubsystem, 9850, 6150, 0.0485, 0.0495)).withTimeout(2.5),

                // shoot preload
                parallel(
                    new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                    new GetToTargetVelocity(shooterSubsystem, 37, 30),
                    new RunFeeder(feederSubsystem, 1),
                    new RunLoader(loaderSubsystem, 1)
                ).withTimeout(1.5), // tune time

                new StraightWithMotionMagic(driveBaseSubsystem, -80.88)
            )
        );
        addCommands(new SetLEDColor(ledSubsystem, limelightSubsystem, driveBaseSubsystem));
    }
    
}
