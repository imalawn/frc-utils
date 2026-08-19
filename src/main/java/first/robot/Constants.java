package first.robot;

import com.ctre.phoenix6.CANBus;
import org.wpilib.framework.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class CANConstants {
    public static final CANBus EXAMPLE_CAN_BUS_0 = CANBus.systemcore(0);
    public static final CANBus EXAMPLE_CAN_BUS_1 = CANBus.systemcore(1);

    public static final int EXAMPLE_ROLLER_LEADER = 0;
    public static final int EXAMPLE_ROLLER_FOLLOWER_1 = 1;
    public static final int EXAMPLE_ROLLER_FOLLOWER_2 = 2;
    public static final int EXAMPLE_ROLLER_FOLLOWER_3 = 3;

    public static final int EXAMPLE_PIVOT_MOTOR = 4;
    public static final int EXAMPLE_PIVOT_ENCODER = 7;

    public static final int EXAMPLE_ELEVATOR_LEADER = 5;
    public static final int EXAMPLE_ELEVATOR_FOLLOWER = 6;
  }
}
