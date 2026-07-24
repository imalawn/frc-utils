package first.robot.util.io.motors;

import static org.wpilib.units.Units.Seconds;

import first.robot.util.RobotUtil;
import first.robot.util.commands.Commands;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.Alert;
import org.wpilib.math.filter.Debouncer;

public abstract class Motor<IOType extends MotorIO, InputsType extends MotorIO.MotorIOInputs> {
  protected final String name;
  protected final IOType io;
  protected final InputsType inputs;
  protected MotorIO.MotorIOMode mode;

  private final BooleanSupplier brakeDurNeutral;

  private final Alert torqueLimitWarning;
  private final Alert tempWarning;
  private final Alert tempFault;
  @Getter protected boolean stalled;
  @Getter protected boolean tempCritical;

  private final RobotUtil.RumbleRequest tempWarnRumble = new RobotUtil.RumbleRequest(0.85, 20);

  protected Motor(
      String name, IOType io, InputsType inputs, BooleanSupplier brakeMode, double currentLimit) {
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

    final RobotUtil.RumbleRequest stallRumble = new RobotUtil.RumbleRequest(0.85, 10);
    new Trigger(() -> inputs.statorCurrentAmps >= currentLimit)
        .debounce(Seconds.of(0.3), Debouncer.DebounceType.kBoth)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      stalled = true;
                      stop();
                      torqueLimitWarning.set(true);
                      RobotUtil.requestDriverRumble(stallRumble);
                      RobotUtil.requestOperatorRumble(stallRumble);
                    })
                .named("Start Stall Protection"))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      stalled = false;
                      torqueLimitWarning.set(false);
                      RobotUtil.stopDriverRumble(stallRumble);
                      RobotUtil.stopOperatorRumble(stallRumble);
                    })
                .named("End Stall Protection"));

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
      RobotUtil.requestOperatorRumble(tempWarnRumble);
    } else {
      tempCritical = false;
      tempFault.set(false);
      tempWarning.set(highestTemp > 60.0);
      RobotUtil.stopOperatorRumble(tempWarnRumble);
    }
  }

  public void runOpenLoop(double volts) {
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
}
