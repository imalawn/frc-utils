package first.robot.util.io.motors.elevator;

import static org.wpilib.units.Units.*;

import first.robot.util.io.motors.Motor;
import first.robot.util.io.motors.MotorIO;
import first.robot.util.subsystems.SubsystemManager;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

public class LinearSystem extends Motor<LinearSystemIO, LinearSystemIOInputsAutoLogged> {
  private final Function<Distance, Angle> distanceToAngle;

  public LinearSystem(
      String name,
      LinearSystemIO io,
      BooleanSupplier brakeMode,
      double currentLimit,
      Function<Distance, Angle> distanceToAngle) {
    super(name, io, new LinearSystemIOInputsAutoLogged(), brakeMode, currentLimit);
    io.configure(true, false);
    this.distanceToAngle = distanceToAngle;
    Logger.recordOutput(name + "/SetpointRad", 0.0);
  }

  public LinearSystem(
      String name,
      LinearSystemIO io,
      BooleanSupplier brakeMode,
      double currentLimit,
      double drumRadiusMeters) {
    this(
        name,
        io,
        brakeMode,
        currentLimit,
        distance -> Radians.of(distance.in(Meters) / drumRadiusMeters));
  }

  public LinearSystem(
      String name, LinearSystemIO io, double currentLimit, double drumRadiusMeters) {
    this(name, io, SubsystemManager::isRobotEnabled, currentLimit, drumRadiusMeters);
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
    Logger.recordOutput(name + "/SetpointRad", angle.in(Radians));
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void runClosedLoop(Distance position) {
    runClosedLoop(distanceToAngle.apply(position));
  }

  public void resetPosition(Angle newPosition) {
    io.resetPosition(newPosition);
  }

  public Angle getPosition() {
    return Radians.of(inputs.positionRad);
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadPerSec);
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }
}
