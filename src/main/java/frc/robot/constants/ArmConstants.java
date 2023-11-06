package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;

public final class ArmConstants {
  public static final double armPower = 0.4;
  public static final double armkf = 0.2;
  public static final double armOffset = 0.185;
  
  public static final double armMomentofInertia = 69.420;

  public static final double armGearing = (double) (64) / 16 * (double) (64) / 14 * 3;
  public static final double armEncoderGearing = (double) 22 / 72;
  public static final double motorEncoderGearing = 114.55 * armEncoderGearing;

  // testing purposes
  public static final double mainArmPowerCoefficient = 0.5;
  public static final double extendedArmPowerCoefficient = 0.3;

  // setpoints for intaking and scoring (arbitrary values until we get arms to
  // tune)
  public static final double intakeSetpoint = 1000;
  public static final double scoreSetpoint = 0;
  public static final double resetSetpoint = 0;

  public static final double maxVelocity = 100;
  public static final double maxAcceleration = 82;

  public static double initialPosition;
  // ff without cone

  
  //TODO Outdated
  public static double withoutConeks = 0.078743;
  public static double withoutConekg = 0.35614;
  public static double withoutConekv = 0.035504;
  public static double withoutConeka = 0.0022254;
  
  //Use this 11/6/2023 Period 2
  public static double ks = 0.10462;
  public static double kg = 0.36345;
  public static double kv = 0.034634;
  public static double ka = 0.0010338;

  // ff with cone
  public static final ArmFeedforward armFeedforward = new ArmFeedforward(ks, kg, kv);
  public static double withConeks = 0.19169;
  public static double withConekg = 0.32053;
  public static double withConekv = 12.345;
  public static double withConeka = 0.58072;

  
  public static double armKp = 0.4;

  public static double armKi = 0;
  public static double armKd = 0;
  public static double armElevatorDelay;
  public static double closedLoopPeakOutput;

  public static enum ArmState {
    RESET(0),
    LOW(30),
    HIGH(60),
    SUBSTATION(80);

    public final double armSetpoint;

    private ArmState(double armSetpoint) {
      this.armSetpoint = armSetpoint;
    }
  }
}
