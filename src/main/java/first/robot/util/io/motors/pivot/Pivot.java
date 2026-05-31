package frc.robot.util.io.motors.pivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.subsystems.RobotStateHandler;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Pivot {
  private final String name;
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private MotorIO.MotorIOMode mode;

  private final BooleanSupplier brakeDurNeutral;

  private final Alert torqueLimitWarning;
  private final Alert tempWarning;
  private final Alert tempFault;
  private boolean motorSafetyEngaged;

  public Pivot(String name, PivotIO io) {
    this(name, io, 120.0);
  }

  public Pivot(String name, PivotIO io, double currentLimit) {
    this(name, io, RobotStateHandler::isEnabled, currentLimit);
  }

  public Pivot(String name, PivotIO io, BooleanSupplier brakeMode, double currentLimit) {
    this.name = name;
    this.io = io;
    this.mode = brakeMode.getAsBoolean() ? MotorIO.MotorIOMode.BRAKE : MotorIO.MotorIOMode.COAST;
    this.brakeDurNeutral = brakeMode;

    // Initialize input arrays
    inputs.followerConnected = new boolean[io.getNumFollowers()];
    inputs.followerTempCelsius = new double[io.getNumFollowers()];

    // Initialize alerts
    torqueLimitWarning =
        new Alert(
            name, "Motor torque limited, disabling to prevent damage", Alert.AlertType.kWarning);
    tempWarning = new Alert(name, "Motor temperature above 60°C", Alert.AlertType.kWarning);
    tempFault =
        new Alert(name, "Motor disabled due to temperature above 75°C", Alert.AlertType.kError);

    new Trigger(() -> inputs.statorCurrentAmps >= currentLimit)
        .debounce(0.3, Debouncer.DebounceType.kBoth)
        .onTrue(
            Commands.runOnce(
                () -> {
                  motorSafetyEngaged = true;
                  io.coast();
                  torqueLimitWarning.set(true);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  motorSafetyEngaged = false;
                  stop();
                  torqueLimitWarning.set(false);
                }));
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
    Logger.recordOutput(name + "/SetpointDeg", -1.0);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void runClosedLoop(double deg) {
    if (motorSafetyEngaged) return;

    io.setPosition(deg);
    mode = MotorIO.MotorIOMode.POSITION_CONTROL;
    Logger.recordOutput(name + "/SetpointDeg", deg);
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
    Logger.recordOutput(name + "/SetpointDeg", -1.0);
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
}
