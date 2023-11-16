/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PIDConstants;

public class DriveWithMotionMagic extends CommandBase {
  
    private DriveBaseSubsystem driveBaseSubsystem;
    private double setpoint;
    private double leftMastOutput;
    private double rightMastOutput;
    private boolean started;
    private long startTime;

    /**
     * 
     * @param driveBaseSubsystem
     * @param setpoint in inches
     */
    public DriveWithMotionMagic(DriveBaseSubsystem driveBaseSubsystem, double setpoint) {
        // this.setpoint = setpoint;
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.setpoint = setpoint;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize(){
        // SmartDashboard.putBoolean("MM Running", false);

        /* factory default + inversions just so nothing acts up */
        driveBaseSubsystem.factoryResetAll();
        driveBaseSubsystem.setAllDefaultInversions();
        driveBaseSubsystem.coast();

        driveBaseSubsystem.getLeftMast().setSelectedSensorPosition(0);
        driveBaseSubsystem.getLeftFollow().setSelectedSensorPosition(0);
        driveBaseSubsystem.getRightMast().setSelectedSensorPosition(0);
        driveBaseSubsystem.getRightFollow().setSelectedSensorPosition(0);
        double maxVelocity = 17000;
        double maxAcceleration = 9000;
        // because sample code 
        driveBaseSubsystem.getLeftMast().configMotionCruiseVelocity(maxVelocity, 0);
        driveBaseSubsystem.getLeftMast().configMotionAcceleration(maxAcceleration, 0);
        driveBaseSubsystem.getLeftFollow().configMotionCruiseVelocity(maxVelocity, 0);
        driveBaseSubsystem.getLeftFollow().configMotionAcceleration(maxAcceleration, 0);

        driveBaseSubsystem.getRightMast().configMotionCruiseVelocity(maxVelocity, 0);
        driveBaseSubsystem.getRightMast().configMotionAcceleration(maxAcceleration, 0);  
        driveBaseSubsystem.getRightFollow().configMotionCruiseVelocity(maxVelocity, 0);
        driveBaseSubsystem.getRightFollow().configMotionAcceleration(maxAcceleration, 0); 

        driveBaseSubsystem.setPIDFConstants(0, driveBaseSubsystem.getLeftMast(), PIDConstants.DriveBaseMotionMagickP, PIDConstants.DriveBaseMotionMagickI, PIDConstants.DriveBaseMotionMagickD, 0);
        driveBaseSubsystem.setPIDFConstants(0, driveBaseSubsystem.getRightMast(), PIDConstants.DriveBaseMotionMagickP, PIDConstants.DriveBaseMotionMagickI, PIDConstants.DriveBaseMotionMagickD, 0);

        double leftSetpoint = UnitConversions.inchesToTicks(setpoint, 3, DriveConstants.driveTrainGearRatio, 2048);
        double rightSetpoint = UnitConversions.inchesToTicks(setpoint, 3, DriveConstants.driveTrainGearRatio, 2048);

        SmartDashboard.putNumber("lSetpoint", leftSetpoint);
        SmartDashboard.putNumber("rSetpoint", rightSetpoint);

        started = false;

        driveBaseSubsystem.getLeftMast().set(ControlMode.MotionMagic, leftSetpoint);
        driveBaseSubsystem.getLeftFollow().set(ControlMode.MotionMagic, leftSetpoint);
        driveBaseSubsystem.getRightMast().set(ControlMode.MotionMagic, rightSetpoint);
        driveBaseSubsystem.getRightFollow().set(ControlMode.MotionMagic, rightSetpoint);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){

        // SmartDashboard.putBoolean("MM Running", true);

        SmartDashboard.putNumber("LM Position", driveBaseSubsystem.getLeftMast().getSelectedSensorPosition(0));
        SmartDashboard.putNumber("RM Position", driveBaseSubsystem.getRightMast().getSelectedSensorPosition(0));
        

        double leftMastOutput = driveBaseSubsystem.getLeftMast().getMotorOutputPercent();
        double rightMastOutput = driveBaseSubsystem.getRightMast().getMotorOutputPercent();
        // SmartDashboard.putNumber("LM Out", leftMastOutput);
        // SmartDashboard.putNumber("RM Out", rightMastOutput);
        // SmartDashboard.putNumber("LM Error", driveBaseSubsystem.getLeftMast().getClosedLoopError());
        // SmartDashboard.putNumber("RM Error", driveBaseSubsystem.getRightMast().getClosedLoopError());
        if(System.currentTimeMillis() - startTime > 1000){
            started = true;
        }


        // SmartDashboard.putBoolean("started", started);
    }

    @Override
    public boolean isFinished(){
        if(started && Math.abs(leftMastOutput) < 0.01 && Math.abs(rightMastOutput) < 0.01){
            Timer.delay(1);
            return true;
        } else{return false;}
    }

    @Override
    public void end(boolean interrupted){
        driveBaseSubsystem.stop();
        driveBaseSubsystem.brake();
        // SmartDashboard.putBoolean("MM Running", false);
    }
}