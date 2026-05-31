package frc.robot.util.io.motors;

public interface MotorIO {
  abstract class MotorIOInputs {
    public boolean connected;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double statorCurrentAmps;
    public double tempCelsius;

    public boolean[] followerConnected;
    public double[] followerTempCelsius;
  }

  enum MotorIOMode {
    COAST,
    BRAKE,
    VOLTAGE_CONTROL,
    POSITION_CONTROL,
    VELOCITY_CONTROL,
  }

  record MechanismConstraints(
      double reduction,
      double moi,
      double radiusMeters,
      double minAngleRads,
      double maxAngleRads,
      double startingAngleRads) {}

  default void setVoltage(double volts) {}

  default void coast() {}

  default void brake() {}

  default int getNumFollowers() {
    return 0;
  }
}
