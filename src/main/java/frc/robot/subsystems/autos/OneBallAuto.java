package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeeder;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.SetLEDColor;

public class OneBallAuto extends ParallelCommandGroup {

    public OneBallAuto(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem,LEDSubsystem ledSubsystem) {
        addCommands(
        );
    }
    
}
