package frc.robot.constants;

public final class DeviceIDs {
  public static enum CanIds {
    // Drivetrain CAN IDs
    leftFalcon1(4),
    rightFalcon1(2),
    leftFalcon2(5),
    rightFalcon2(3),

    // Arm CAN IDs
    armMain1(12),
    armMain2(14),
    armExtended(51),

    // Gripper CAN IDs
    gripperSRX(57), // change later

    // Elevator CAN IDs
    mainElevatorMotor(13),

    // Gyro CAN IDss
    pigeon(0),
    extendedPigeon(51),

    wristSpark(100), // unknown
    ;

    public final int id;

    private CanIds(int id) {
      this.id = id;
    }
  }

  public static enum SensorIds {

    // Beambreak DIO IDs
    beambreak(2),

    // Limit switch DIO IDs
    limitswitch(0),
    ;

    public final int id;

    private SensorIds(int id) {
      this.id = id;
    }
  }
}
