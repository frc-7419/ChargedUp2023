package frc.robot.subsystems.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {

  private AHRS ahrs;

  public GyroSubsystem() {
        try {
			/* Communicate w/navX-MXP via the MXP SPI Bus (use mini USB to USB A cable)   
			   Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or S     
			   See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
               ahrs = new AHRS(SerialPort.Port.kMXP); 
		} catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true); 
        }
    }

    public double getGyroAngle() {
        return ahrs.getAngle();
    }
    public Rotation2d getRotation2d(){
        return ahrs.getRotation2d();
    }
    @Override
    public void periodic() {
         SmartDashboard.putNumber("robot pitch", ahrs.getPitch());
    }
}