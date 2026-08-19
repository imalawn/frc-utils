package first.robot.util.io.motors;

import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Alert;

public abstract class Motor<IOType extends MotorIO, InputsType extends MotorIO.MotorIOInputs> {
  protected final String name;
  protected final IOType io;
  protected final InputsType inputs;
  protected MotorIO.MotorIOMode mode;

  private final BooleanSupplier brakeDurNeutral;

  private final Alert torqueLimitWarning;
  private final Alert tempWarning;
  private final Alert tempFault;
  @Getter protected boolean tempCritical;

  protected Motor(String name, IOType io, InputsType inputs, BooleanSupplier brakeMode) {
    this.name = name;
    this.io = io;
    this.inputs = inputs;
    this.mode = brakeMode.getAsBoolean() ? MotorIO.MotorIOMode.BRAKE : MotorIO.MotorIOMode.COAST;
    this.brakeDurNeutral = brakeMode;

    // Initialize input arrays
    inputs.followerConnected = new boolean[io.getNumFollowers()];
    inputs.followerTempCelsius = new double[io.getNumFollowers()];

    // Initialize alerts
    torqueLimitWarning =
        new Alert(name, "Motor torque limited, disabling to prevent damage", Alert.Level.MEDIUM);
    tempWarning = new Alert(name, "Motor temperature above 60°C", Alert.Level.MEDIUM);
    tempFault = new Alert(name, "Motor disabled due to temperature above 75°C", Alert.Level.HIGH);

    // Use correct brake/coast state
    stop();
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void periodic() {
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
    Logger.recordOutput(name + "/MotorMode", mode);
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
