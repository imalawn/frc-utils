package frc.robot.util.io.motors.pivot;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.io.motors.MotorIO;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO extends MotorIO {
  @AutoLog
  class PivotIOInputs extends MotorIOInputs {
    public double positionDeg;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void setPosition(double deg) {}

  default void resetPosition(Angle angle) {}
}
