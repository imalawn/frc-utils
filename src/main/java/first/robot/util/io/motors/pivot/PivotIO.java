package first.robot.util.io.motors.pivot;

import first.robot.util.io.motors.MotorIO;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;

public interface PivotIO extends MotorIO {
  @AutoLog
  class PivotIOInputs extends MotorIOInputs {
    public double positionDeg;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void setPosition(Angle angle) {}

  default void setPosition(int slot, Angle angle) {
    setPosition(angle);
  }

  default void resetPosition(Angle angle) {}
}
