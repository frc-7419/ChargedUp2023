package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class IntakeSubsystem extends SubsystemBase{
    private CANSparkMax intake;

    public IntakeSubsystem() {
        intake = new CANSparkMax(CanIds.intakeSpark.id, MotorType.kBrushless);
        intake.setInverted(true);
        intake.setIdleMode(IdleMode.kCoast);
        intake.burnFlash();
    }

    @Override
    public void periodic() {}

    public void setPower(double power) {
        intake.set(power);
    }

    public CANSparkMax getIntakeMotor() {
        return intake;
    } 
}