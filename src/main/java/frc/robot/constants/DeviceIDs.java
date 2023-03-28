package frc.robot.constants;

public final class DeviceIDs {
  public static enum CanIds {
    // Drivetrain CAN IDs
    leftFalcon1(4),
    rightFalcon1(2),
    leftFalcon2(5),
    rightFalcon2(3),

    // Arm CAN IDs
    armFalcon(13),

    // Gripper CAN IDs
    gripperNeo(15), // change later

    // Elevator CAN IDs
    mainElevatorMotor(12
    ),

    // Gyro CAN IDss
    pigeon(0),
    extendedPigeon(51),

    wristSpark(14), // unknown
    ;

    public final int id;

    private CanIds(int id) {
      this.id = id;
    }
  }

  public static enum SensorIds {

    // Beambreak DIO ID
    beamBreak(2),

    // Limit switch DIO ID
    limitSwitch(0),

    // Arm Through Bore Encoder DIO ID
    armAbsoluteEncoder(3),
    armRelativeEncoder1(5),
    armRelativeEncoder2(4),
    elevatorAbsoluteEncoder(3);

    // Wrist Through Bore Encoder DIO ID
    // wristAbsoluteEncoder(1), 
    // elevatorAbsoluteEncoder(1);

    public final int id;

    private SensorIds(int id) {
      this.id = id;
    }
  }
}
