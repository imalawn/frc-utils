package first.robot.util.io.motors;

import first.robot.util.io.sensors.EncoderIO;
import first.robot.util.io.sensors.EncoderIOInputsAutoLogged;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Alert;
import org.wpilib.units.measure.Angle;

public abstract class Motor<T extends MotorIO, U extends MotorIO.MotorIOInputs> {
  protected final String name;
  protected final String modeLogKey;
  protected final T io;
  protected final U inputs;
  protected final EncoderIO encoderIO;
  protected final EncoderIOInputsAutoLogged encoderInputs = new EncoderIOInputsAutoLogged();
  protected MotorIO.MotorIOMode mode;

  private final BooleanSupplier brakeDurNeutral;

  private final Alert tempWarning;
  private final Alert tempFault;
  @Getter protected boolean tempCritical;

  protected Motor(String name, T io, U inputs, EncoderIO encoderIO, BooleanSupplier brakeMode) {
    this.name = name;
    this.modeLogKey = name + "/MotorMode";
    this.io = io;
    this.inputs = inputs;
    this.encoderIO = encoderIO;
    this.brakeDurNeutral = brakeMode;

    // Initialize input arrays
    inputs.followerConnected = new boolean[io.getNumFollowers()];
    inputs.followerTempCelsius = new double[io.getNumFollowers()];

    // Initialize alerts
    tempWarning = new Alert(name, "Motor temperature above 60°C", Alert.Level.MEDIUM);
    tempFault = new Alert(name, "Motor disabled due to temperature above 75°C", Alert.Level.HIGH);

    // Use correct brake/coast state
    stop();
    Logger.recordOutput(modeLogKey, mode);
  }

  /**
   * Updates motor and encoder inputs and handles motor safety features. Call this first in your
   * subsystem {@code periodic()} method.
   */
  public void periodic() {
    encoderIO.updateInputs(encoderInputs);
    Logger.processInputs(name, encoderInputs);

    double highestTemp = inputs.tempCelsius;
    for (double temp : inputs.followerTempCelsius) {
      highestTemp = Math.max(highestTemp, temp);
    }
    if (highestTemp > 75.0) {
      tempCritical = true;
      stop();
      tempFault.set(true);
    } else {
      tempCritical = false;
      tempFault.set(false);
      tempWarning.set(highestTemp > 60.0);
    }
  }

  public void runVoltage(double volts) {
    if (tempCritical) return;

    io.setVoltage(volts);
    mode = MotorIO.MotorIOMode.VOLTAGE_CONTROL;
    Logger.recordOutput(modeLogKey, mode);
  }

  public void stop() {
    if (brakeDurNeutral.getAsBoolean()) {
      io.brake();
      mode = MotorIO.MotorIOMode.BRAKE;
    } else {
      io.coast();
      mode = MotorIO.MotorIOMode.COAST;
    }
    Logger.recordOutput(modeLogKey, mode);
  }

  public Angle getAbsolutePosition(boolean refresh) {
    if (refresh) {
      encoderIO.updateInputs(encoderInputs);
      Logger.processInputs(name, encoderInputs);
    }
    return encoderInputs.absolutePosition;
  }

  public Angle getAbsolutePosition() {
    return encoderInputs.absolutePosition;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public double getAppliedVoltage() {
    return inputs.appliedVoltage;
  }

  public double getStatorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  public double getTempCelsius() {
    return inputs.tempCelsius;
  }

  public boolean[] getFollowerConnected() {
    return inputs.followerConnected;
  }

  public double[] getFollowerTempCelsius() {
    return inputs.followerTempCelsius;
  }
}
