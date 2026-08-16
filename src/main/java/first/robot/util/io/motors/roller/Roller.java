package first.robot.util.io.motors.roller;

import static org.wpilib.units.Units.RotationsPerSecond;

import first.robot.util.io.motors.Motor;
import first.robot.util.io.motors.MotorIO;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.units.measure.AngularVelocity;

public class Roller extends Motor<RollerIO, RollerIOInputsAutoLogged> {
  public Roller(String name, RollerIO io, BooleanSupplier brakeMode, double currentLimit) {
    super(name, io, new RollerIOInputsAutoLogged(), brakeMode, currentLimit);
    io.configure(false, true);
    Logger.recordOutput(name + "/SetpointRPS", 0.0);
  }

  public Roller(String name, RollerIO io, double currentLimit) {
    this(name, io, () -> false, currentLimit);
  }

  public Roller(String name, RollerIO io) {
    this(name, io, 120.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    super.periodic();
  }

  public void runClosedLoop(double rps) {
    if (tempCritical) return;

    io.setVelocity(rps);
    mode = MotorIO.MotorIOMode.VELOCITY_CONTROL;
    Logger.recordOutput(name + "/SetpointRPS", rps);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(inputs.velocityRPS);
  }

  public double getVelocityRPS() {
    return inputs.velocityRPS;
  }
}
