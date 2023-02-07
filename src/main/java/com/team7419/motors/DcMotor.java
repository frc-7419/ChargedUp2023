package com.team7419.motors;

import com.team7419.math.*;

/**
 * simulation motor based on 971's motor model essentially modelling motor as generator + resistor
 */
public class DcMotor {

  private double kT;
  private double kV;
  private double kInternalResistance;

  protected double m_position;
  protected double m_velocity;
  protected double m_current;

  /** constructs a DcMotor based on a given MotorType */
  public DcMotor(MotorType motorType) {

    /** sets the motor constants kT and kV based on motorType */
    switch (motorType.value) {
        /** cim */
      case 0:
        kInternalResistance = getkInternalResistance(131);
        kV = getkV(kInternalResistance, 5330, 2.7);
        kT = getkT(2.41, 131);
        /** mini cim */
      case 1:
        kInternalResistance = getkInternalResistance(89);
        kV = getkV(kInternalResistance, 5840, 3.0);
        kT = getkT(1.41, 89);
        /** 775 pro */
      case 2:
        kInternalResistance = getkInternalResistance(134);
        kV = getkV(kInternalResistance, 18730, 0.7);
        kT = getkT(.71, 134);
    }
  }

  /**
   * pretty much only for use setting up a transmission but like eh
   *
   * @param kV can be computed using getkV
   * @param kT can be computed using getkT
   * @param kInternalResistance can be computed using getkInternalResistance
   */
  public DcMotor(double kV, double kT, double kInternalResistance) {
    this.kV = kV;
    this.kInternalResistance = kInternalResistance;
    this.kT = kT;
  }

  /**
   * def minorly stealing from 254 but like ehh
   *
   * @param motor instantiate DcMotor with appropriate MotorType and pass through
   * @param numOfMotors number of motors in gearbox
   * @param gearReduction gear ratio
   * @return a DcMotor object that'll behave like the whole transmission
   */
  public static DcMotor makeTransmission(DcMotor motor, int numOfMotors, double gearReduction) {
    return new DcMotor(
        motor.kV / gearReduction, // scale kV by number of motors
        numOfMotors * gearReduction * motor.kT, // scale kT by number of motors, gearing
        motor.kInternalResistance
            / numOfMotors); // scale resistance by # of motors (think: parallel circuits)
  }

  /** set values of position, velocity, current to whatever you want */
  public void reset(double position, double velocity, double current) {
    m_position = position;
    m_current = current;
    m_velocity = velocity;
  }

  /**
   * @param voltage voltage given to motor
   * @param load weight on mechanism
   * @param externalTorque i.e. gravity
   * @param timestep just go w 0.01 for now
   */
  public void step(double voltage, double load, double externalTorque, double timestep) {
    double acceleration =
        (voltage - m_velocity / kV) * kT / (kInternalResistance * load) + externalTorque / load;
    m_velocity += acceleration * timestep;
    m_position += m_velocity * timestep + .5 * acceleration * timestep * timestep;
    m_current = load * acceleration * Math.signum(voltage) / kT;
  }

  public double getPosition() {
    return m_position;
  }

  public double getVelocity() {
    return m_velocity;
  }

  public double getCurrent() {
    return m_current;
  }

  private double getkInternalResistance(double kPeakCurrent) {
    return 12 / kPeakCurrent;
  }

  /**
   * @param kPeakCurrent in Amps, current at stall torque
   * @param kFreeSpeed in rpm, when there's no load on motor
   * @param kFreeCurrent in Amps, current at free speed
   * @return voltage constant of the motor for use in calculations
   */
  private double getkV(double internalResistance, double freeSpeed, double freeCurrent) {
    return (UnitConversions.rpmToRadPerSec(freeSpeed) + internalResistance * freeCurrent) / 12;
  }

  /**
   * @param kStallTorque in N*m, when motor is stalled
   * @param kStallCurrent in Amps, current at stall torque
   * @return torque constant of the motor for use in calculations
   */
  private double getkT(double stallTorque, double stallCurrent) {
    return stallTorque / stallCurrent;
  }
}
