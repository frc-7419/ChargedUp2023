package com.team7419;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class MotorGroup {

  private TalonFX master;
  private VictorSPX[] victors;

  /**
   * @param master
   * @param victors
   */
  public MotorGroup(TalonFX master, VictorSPX... victors) {
    this.master = master;
    this.victors = victors;
    followMaster();
  }

  public void followMaster() {
    for (VictorSPX victor : this.victors) {
      victor.follow(this.master);
    }
  }

  public void setInverted(boolean isInverted) {
    master.setInverted(isInverted);
    for (VictorSPX victor : this.victors) {
      victor.setInverted(isInverted);
    }
  }

  public void setPower(double power) {
    master.set(ControlMode.PercentOutput, power);
  }
}
