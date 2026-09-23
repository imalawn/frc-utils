package first.robot.util.io.sensors.lasercan;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface DistanceSensorIO {
  @AutoLog
  class DistanceSensorIOInputs {
    public boolean connected;
    public boolean measurementValid;
    public double distanceMillimeters;
  }

  static DistanceSensorIO beambreakSim(BooleanSupplier beambreak, double threshold) {
    return inputs -> {
      inputs.connected = true;
      inputs.measurementValid = true;
      inputs.distanceMillimeters = beambreak.getAsBoolean() ? threshold - 1 : threshold + 1;
    };
  }

  void updateInputs(DistanceSensorIOInputs inputs);
}
