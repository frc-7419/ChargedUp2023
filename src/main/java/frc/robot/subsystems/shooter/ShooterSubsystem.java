package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team7419.InterpolatedTreeMap;
import com.team7419.TalonFuncs;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX bottomFalcon;
    private TalonFX topFalcon;
    
    private SimpleMotorFeedforward topFeedforward;
    private SimpleMotorFeedforward bottomFeedforward;

    private double bottomTargetVelocity = 0; // MPS
    private double topTargetVelocity = 0; 

    private double topTargetRawVelocity = 0;
    private double bottomTargetRawVelocity = 0;

    private double rawVelocityThreshold = 50; 

    private double maxVoltage = 11;

    public ShooterSubsystem(){
        bottomFalcon = new TalonFX(CanIds.bottomShooterFalcon.id);
        topFalcon = new TalonFX(CanIds.topShooterFalcon.id);

        // bottomFalcon.configFactoryDefault();
        // topFalcon.configFactoryDefault();

        bottomFalcon.configVoltageCompSaturation(maxVoltage); // "full output" will now scale to 11 Volts for all control modes when enabled.
        bottomFalcon.enableVoltageCompensation(true); // turn on/off feature

        topFalcon.configVoltageCompSaturation(maxVoltage);
        topFalcon.enableVoltageCompensation(true);

        bottomFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        topFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        bottomFalcon.setInverted(false);
        topFalcon.setInverted(false);
        
        this.configShooterOutputs();

        topFeedforward = new SimpleMotorFeedforward(RobotConstants.TopShooterKs, RobotConstants.TopShooterKv);
        bottomFeedforward = new SimpleMotorFeedforward(RobotConstants.BottomShooterKs, RobotConstants.BottomShooterKv);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("tRV", getCurrentTopRawVelocity());
        SmartDashboard.putNumber("bRV", getCurrentBottomRawVelocity());
        SmartDashboard.putNumber("tMPS", topFalcon.getSelectedSensorVelocity(0) * 10 * (1/2048) * (1/RobotConstants.RotationsPerMeter));
        SmartDashboard.putNumber("bMPS", bottomFalcon.getSelectedSensorVelocity(0) * 10 * (1/2048) * (1/RobotConstants.RotationsPerMeter));

        SmartDashboard.putNumber("tMPS", getCurrentTopVelocity());
        SmartDashboard.putNumber("bMPS", getCurrentBottomVelocity());

        SmartDashboard.putNumber("tError", getCurrentTopVelocity() - topTargetVelocity);
        SmartDashboard.putNumber("bError", getCurrentBottomVelocity() - bottomTargetVelocity);
        SmartDashboard.putBoolean("is on target", bothOnTarget());
    }


    public void configShooterOutputs() {
        topFalcon.configNominalOutputForward(0, 0);
        bottomFalcon.configNominalOutputForward(0, 0);

        topFalcon.configNominalOutputReverse(0, 0);
		bottomFalcon.configNominalOutputReverse(0, 0);
        
        topFalcon.configPeakOutputForward(1, 0);
		bottomFalcon.configPeakOutputForward(1, 0);
        
        topFalcon.configPeakOutputReverse(-1, 0);
        bottomFalcon.configPeakOutputReverse(-1, 0);
    }

    public void setTopClosedLoopVelocity(double velocityMetersPerSecond) {
        this.topTargetVelocity = velocityMetersPerSecond;
        this.topTargetRawVelocity = velocityMetersPerSecond * RobotConstants.RotationsPerMeter * 2048 * 0.1;
        topFalcon.set(ControlMode.Velocity, velocityMetersPerSecond * RobotConstants.RotationsPerMeter * 2048 * 0.1, DemandType.ArbitraryFeedForward, topFeedforward.calculate(velocityMetersPerSecond) / maxVoltage);
    }
    public void setBottomClosedLoopVelocity(double velocityMetersPerSecond) {
        this.bottomTargetVelocity = velocityMetersPerSecond;
        this.bottomTargetRawVelocity = velocityMetersPerSecond * RobotConstants.RotationsPerMeter * 2048 * 0.1;
        bottomFalcon.set(ControlMode.Velocity, velocityMetersPerSecond * RobotConstants.RotationsPerMeter * 2048 * 0.1, DemandType.ArbitraryFeedForward, bottomFeedforward.calculate(velocityMetersPerSecond) / maxVoltage);
    }

    public void setTopPIDF(double kP, double kI, double kD, double kF){
        TalonFuncs.setPIDFConstants(0, topFalcon, kP, kI, kD, kF);
    }

    public void setBottomPIDF(double kP, double kI, double kD, double kF) {
        TalonFuncs.setPIDFConstants(0, bottomFalcon, kP, kI, kD, kF);
    }

    public boolean topOnTarget() {
        return Math.abs(getCurrentTopRawVelocity() - topTargetRawVelocity) < rawVelocityThreshold;
    }

    public boolean bottomOnTarget() {
        return Math.abs(getCurrentBottomRawVelocity() - bottomTargetRawVelocity) < rawVelocityThreshold;
    }

    public boolean bothOnTarget() {
        return topOnTarget() && bottomOnTarget();
    }

    public double getTopError() {
        return getCurrentTopRawVelocity() - topTargetRawVelocity;
    }

    public double getBottomError() {
        return getCurrentBottomRawVelocity() - bottomTargetRawVelocity;
    }

    public void setTopVoltage(double voltage) {
        topFalcon.set(ControlMode.PercentOutput, voltage/11);
    }
    public void setBottomVoltage(double voltage) {
        bottomFalcon.set(ControlMode.PercentOutput, voltage/11);
    }
    public void setBothVoltage(double voltage) {
        topFalcon.set(ControlMode.PercentOutput, voltage/11);
        bottomFalcon.set(ControlMode.PercentOutput, voltage/11);
    }

    public void setTopPower(double power) {
        topFalcon.set(ControlMode.PercentOutput, power);
    }
    public void setBottomPower(double power) {
        bottomFalcon.set(ControlMode.PercentOutput, power);
    }
    public void setBothPower(double power) {
        setTopPower(power);
        setBottomPower(power);
    }
    public void off() {
        setBothPower(0);
    }

    public double getTopOutputVoltage(){return topFalcon.getMotorOutputVoltage();}
    public double getBottomOutputVoltage(){return bottomFalcon.getMotorOutputVoltage();}

    public void configInterpolatedTreeMapReferencePoints(Double[][] referencePoints, InterpolatedTreeMap interpolatedTreeMap) {
        for (Double[] i : referencePoints) {
            interpolatedTreeMap.put(i[0], i[1]);
        }
    }

    public double getCurrentTopRawVelocity(){return topFalcon.getSelectedSensorVelocity(0);}
    public double getCurrentBottomRawVelocity(){return bottomFalcon.getSelectedSensorVelocity(0);}

    public double getCurrentTopVelocity(){return topFalcon.getSelectedSensorVelocity(0) * 10 * (1/2048) * (1/RobotConstants.RotationsPerMeter);}
    public double getCurrentBottomVelocity(){return bottomFalcon.getSelectedSensorVelocity(0) * 10 * (1/2048) * (1/RobotConstants.RotationsPerMeter);}

    public double getTopPercentOutput() {return topFalcon.getMotorOutputPercent();}
    public double getBottomPercentOutput() {return bottomFalcon.getMotorOutputPercent();}

    public TalonFX getTopTalon(){return topFalcon;}
    public TalonFX getBottomTalon(){return bottomFalcon;}

}