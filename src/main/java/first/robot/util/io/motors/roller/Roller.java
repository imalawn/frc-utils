package frc.robot.util.io.motors.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.util.io.motors.MotorIO;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Roller {
  private final String name;
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private MotorIO.MotorIOMode mode;

  private final BooleanSupplier brakeDurNeutral;

  private final Alert tempWarning;
  private final Alert tempFault;
  private boolean motorSafetyEngaged;

  public Roller(String name, RollerIO io) {
    this(name, io, () -> false);
  }

  public Roller(String name, RollerIO io, BooleanSupplier brakeMode) {
    this.name = name;
    this.io = io;
    this.mode = brakeMode.getAsBoolean() ? MotorIO.MotorIOMode.BRAKE : MotorIO.MotorIOMode.COAST;
    this.brakeDurNeutral = brakeMode;

    // Initialize input arrays
    inputs.followerConnected = new boolean[io.getNumFollowers()];
    inputs.followerTempCelsius = new double[io.getNumFollowers()];

    // Initialize alerts
    tempWarning = new Alert(name, "Motor temperature above 60°C", Alert.AlertType.kWarning);
    tempFault =
        new Alert(name, "Motor disabled due to temperature above 75°C", Alert.AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    double highestTemp = inputs.tempCelsius;
    for (double temp : inputs.followerTempCelsius) {
      highestTemp = Math.max(highestTemp, temp);
    }
    if (highestTemp > 75.0) {
      motorSafetyEngaged = true;
      stop();
      tempFault.set(true);
    } else {
      motorSafetyEngaged = false;
      tempFault.set(false);
      tempWarning.set(highestTemp > 60.0);
    }
  }

  public void runOpenLoop(double volts) {
    if (motorSafetyEngaged) return;

    io.setVoltage(volts);
    mode = MotorIO.MotorIOMode.VOLTAGE_CONTROL;
    Logger.recordOutput(name + "/SetpointRPS", -1.0);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void runClosedLoop(double rps) {
    if (motorSafetyEngaged) return;

    io.setVelocity(rps);
    mode = MotorIO.MotorIOMode.VELOCITY_CONTROL;
    Logger.recordOutput(name + "/SetpointRPS", rps);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void stop() {
    if (brakeDurNeutral.getAsBoolean()) {
      io.brake();
      mode = MotorIO.MotorIOMode.BRAKE;
    } else {
      io.coast();
      mode = MotorIO.MotorIOMode.COAST;
    }
    Logger.recordOutput(name + "/SetpointRPS", -1.0);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(inputs.velocityRPS);
  }

  public double getVelocityRPS() {
    return inputs.velocityRPS;
  }
}
