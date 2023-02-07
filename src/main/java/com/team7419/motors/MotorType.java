package com.team7419.motors;

public enum MotorType {

  /** CIM motor */
  Cim(0),

  /** Mini CIM motor */
  MiniCim(1),

  /** 775 pro */
  Pro775(2);

  public final int value;

  /**
   * Create MotorType of initValue
   *
   * @param initValue value of MotorType
   */
  MotorType(int initValue) {
    this.value = initValue;
  }
}
