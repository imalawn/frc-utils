package first.robot.util.io.motors.pivot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;

import first.robot.util.io.motors.Motor;
import first.robot.util.io.motors.MotorIO;
import first.robot.util.subsystems.SubsystemManager;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class Pivot extends Motor<PivotIO, PivotIOInputsAutoLogged> {
  public Pivot(String name, PivotIO io, BooleanSupplier brakeMode, double currentLimit) {
    super(name, io, new PivotIOInputsAutoLogged(), brakeMode, currentLimit);
    io.configure(true, false);
    Logger.recordOutput(name + "/SetpointDeg", 0.0);
  }

  public Pivot(String name, PivotIO io, double currentLimit) {
    this(name, io, SubsystemManager::isRobotEnabled, currentLimit);
  }

  public Pivot(String name, PivotIO io) {
    this(name, io, 120.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    super.periodic();
  }

  public void runClosedLoop(Angle angle) {
    if (tempCritical) return;

    io.setPosition(angle);
    mode = MotorIO.MotorIOMode.POSITION_CONTROL;
    Logger.recordOutput(name + "/SetpointDeg", angle.in(Degrees));
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void resetPosition(Angle newPosition) {
    io.resetPosition(newPosition);
  }

  public Angle getPosition() {
    return Degrees.of(inputs.positionDeg);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }

  public AngularVelocity getVelocity() {
    return DegreesPerSecond.of(inputs.velocityDegPerSec);
  }

  public double getVelocityDegPerSec() {
    return inputs.velocityDegPerSec;
  }
}
