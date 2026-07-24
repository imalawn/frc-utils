package first.robot.util.io.sensors;

import static org.wpilib.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;

@FunctionalInterface
public interface EncoderIO {
  @AutoLog
  class EncoderIOInputs {
    public boolean connected = false;
    public Angle position = Rotations.zero();
  }

  void updateInputs(EncoderIOInputs inputs);
}
