package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSolenoidSubsystem extends SubsystemBase{
    private Solenoid intakeSolenoid; 

    public IntakeSolenoidSubsystem() {
        intakeSolenoid = new Solenoid(30, PneumaticsModuleType.REVPH, 15);
    }

    @Override
    public void periodic() {}

    public void actuateSolenoid() {
        intakeSolenoid.set(true);
    }
    
    public void retractSolenoid() {
        intakeSolenoid.set(false);
    }

    public void toggleSolenoid() {
        intakeSolenoid.toggle();
    }

    public Solenoid getIntakeSolenoid() {
        return intakeSolenoid;
    }
}