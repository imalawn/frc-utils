package first.robot.util.io.motors.elevator;

import first.robot.util.io.motors.MotorIO;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;

public interface LinearSystemIO extends MotorIO {
  @AutoLog
  class LinearSystemIOInputs extends MotorIOInputs {
    public double positionRad;
    public double velocityRadPerSec;
  }

  default void updateInputs(LinearSystemIOInputs inputs) {}

  default void setPosition(Angle angle) {}

  default void setPosition(int slot, Angle angle) {
    setPosition(angle);
  }

  default void setVelocity(double rps) {}

  default void resetPosition(Angle angle) {}
}
