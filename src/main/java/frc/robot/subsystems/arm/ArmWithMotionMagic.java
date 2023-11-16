/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PIDConstants;

public class ArmWithMotionMagic extends CommandBase {
  
    private ArmSubsystem armSubsystem;
    private double setpointInTicks;
    private boolean started;
    private double armPower;
    private double startTime;

    /**
     * 
     * @param armSubsystem
     * @param setpoint in inches
     */
    public ArmWithMotionMagic(ArmSubsystem armSubsystem, double setpoint) {
        // this.setpoint = setpoint;
        this.armSubsystem = armSubsystem;
        this.setpointInTicks = -34693 + setpoint/360 * ArmConstants.motorEncoderGearing * 2048;
        // this.setpointInTicks = 86000;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        // SmartDashboard.putBoolean("MM Running", false);

        /* factory default + inversions just so nothing acts up */
        armSubsystem.coast();

        double maxVelocity = 1.2* ArmConstants.maxVelocity/360 * ArmConstants.motorEncoderGearing * 2048;
        double maxAcceleration =1.2* ArmConstants.maxAcceleration/360 * ArmConstants.motorEncoderGearing * 2048;
        // because sample code 
        armSubsystem.getArmMotor().configMotionCruiseVelocity(maxVelocity, 0);
        armSubsystem.getArmMotor().configMotionAcceleration(maxAcceleration, 0);
 
       

        armSubsystem.setPIDFConstants(0, armSubsystem.getArmMotor(), PIDConstants.ArmMotionMagickP, PIDConstants.ArmMotionMagickI, PIDConstants.ArmMotionMagickD, 0);
       

        

        SmartDashboard.putNumber("Arm Setpoint In Ticks", setpointInTicks);


        armSubsystem.getArmMotor().set(ControlMode.MotionMagic, setpointInTicks);
        

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){

        // SmartDashboard.putBoolean("MM Running", true);

        SmartDashboard.putNumber("Arm Position", armSubsystem.getArmMotor().getSelectedSensorPosition(0));
        

        this.armPower = armSubsystem.getArmMotor().getMotorOutputPercent();


        // SmartDashboard.putBoolean("started", started);
    }

    @Override
    public boolean isFinished(){
        if (Math.abs(armSubsystem.getArmMotor().getSelectedSensorPosition() - setpointInTicks)<2048){
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted){
        armSubsystem.setPower(0);
        armSubsystem.brake();
        // SmartDashboard.putBoolean("MM Running", false);
    }
}