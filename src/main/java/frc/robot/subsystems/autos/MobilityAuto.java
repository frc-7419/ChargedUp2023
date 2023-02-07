package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;

public class MobilityAuto extends ParallelCommandGroup {

    public MobilityAuto(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
        addCommands(
                new StraightWithMotionMagic(driveBaseSubsystem, 5));
    }

}
