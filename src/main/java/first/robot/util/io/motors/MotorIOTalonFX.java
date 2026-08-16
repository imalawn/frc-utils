package first.robot.util.io.motors;

import static first.robot.util.PhoenixUtil.tryUntilOk;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import first.robot.util.PhoenixUtil;
import first.robot.util.io.motors.elevator.LinearSystemIO;
import first.robot.util.io.motors.pivot.PivotIO;
import first.robot.util.io.motors.roller.RollerIO;
import first.robot.util.io.sensors.EncoderIOCANcoder;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.wpilib.system.Notifier;
import org.wpilib.units.measure.*;

public class MotorIOTalonFX implements AutoCloseable, RollerIO, PivotIO, LinearSystemIO {
  protected final TalonFX leader;
  protected final TalonFX[] followers;

  private VelocityVoltage velocityRequest;
  private List<Consumer<Angle>> positionRequests;
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final CoastOut coastRequest = new CoastOut();
  private final StaticBrake brakeRequest = new StaticBrake();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;
  private final BaseStatusSignal[] followerTemps;

  private volatile Angle angleResetVal = Rotations.zero();
  private Notifier resetPosition;

  private boolean positionConfigured;
  private boolean velocityConfigured;

  public MotorIOTalonFX(
      CANBus canbus,
      int id,
      TalonFXConfiguration config,
      int[] followerIds,
      MotorAlignmentValue[] followerAlignments) {
    // Instantiate motors
    leader = new TalonFX(id, canbus);
    followers = new TalonFX[followerIds.length];
    for (int i = 0; i < followers.length; i++) {
      followers[i] = new TalonFX(followerIds[i], canbus);
    }
    // Configure motors
    tryUntilOk(5, () -> leader.getConfigurator().apply(config));
    for (TalonFX follower : followers) {
      tryUntilOk(5, () -> follower.getConfigurator().apply(config));
    }
    // Create status signals
    position = leader.getPosition();
    velocity = leader.getVelocity();
    voltage = leader.getMotorVoltage();
    statorCurrent = leader.getStatorCurrent();
    temp = leader.getDeviceTemp();
    followerTemps = new BaseStatusSignal[followers.length];
    for (int i = 0; i < followerTemps.length; i++) {
      followerTemps[i] = followers[i].getDeviceTemp();
    }
    // Register status signals
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, voltage, statorCurrent, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, followerTemps);
    leader.optimizeBusUtilization();
    ParentDevice.optimizeBusUtilizationForAll(followers);
    PhoenixUtil.registerSignals(canbus, voltage, statorCurrent, temp);
    PhoenixUtil.registerSignals(canbus, followerTemps);
    tryUntilOk(5, () -> leader.setPosition(0));
    // Set follower behavior
    if (followers.length != followerAlignments.length) {
      throw new IllegalArgumentException("Every follower motor must have an alignment value!");
    }
    for (int i = 0; i < followers.length; i++) {
      followers[i].setControl(new Follower(leader.getDeviceID(), followerAlignments[i]));
    }
  }

  public MotorIOTalonFX(CANBus canbus, int id, TalonFXConfiguration config) {
    this(canbus, id, config, new int[0], new MotorAlignmentValue[0]);
  }

  @Override
  public void configure(boolean positionControl, boolean velocityControl) {
    if (positionControl) {
      configurePositionControl();
    }
    if (velocityControl) {
      configureVelocityControl();
    }
  }

  private void updateMotorInputs(MotorIOInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(voltage, statorCurrent, temp);
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();

    for (int i = 0; i < followerTemps.length; i++) {
      inputs.followerConnected[i] = followerTemps[i].getStatus().isOK();
      inputs.followerTempCelsius[i] = followerTemps[i].getValueAsDouble();
    }
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocityRPS = velocity.getValueAsDouble();
    updateMotorInputs(inputs);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionDeg = position.getValue().in(Degrees);
    updateMotorInputs(inputs);
  }

  @Override
  public void updateInputs(LinearSystemIOInputs inputs) {
    inputs.positionRad = position.getValue().in(Radians);
    inputs.velocityRadPerSec = velocity.getValue().in(RadiansPerSecond);
    updateMotorInputs(inputs);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(Angle angle) {
    positionRequests.getFirst().accept(angle);
  }

  @Override
  public void setPosition(int slot, Angle angle) {
    positionRequests.get(slot).accept(angle);
  }

  @Override
  public void setVelocity(double rps) {
    leader.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void coast() {
    leader.setControl(coastRequest);
  }

  @Override
  public void brake() {
    leader.setControl(brakeRequest);
  }

  @Override
  public void resetPosition(Angle angle) {
    if (!positionConfigured) return;
    angleResetVal = angle;
    resetPosition.startSingle(0);
  }

  private void configurePositionControl() {
    if (positionConfigured) return;
    usingControlRequest(new PositionVoltage(0));
    position.setUpdateFrequency(100.0);
    PhoenixUtil.registerSignals(leader.getNetwork(), position);
    resetPosition = new Notifier(() -> leader.setPosition(angleResetVal));
    positionConfigured = true;
  }

  private void configureVelocityControl() {
    if (velocityConfigured) return;
    velocityRequest = new VelocityVoltage(0);
    velocity.setUpdateFrequency(100.0);
    PhoenixUtil.registerSignals(leader.getNetwork(), velocity);
    velocityConfigured = true;
  }

  private void addControlRequestPrivate(Consumer<Angle> request) {
    if (positionRequests == null) {
      positionRequests = new ArrayList<>(1);
    }
    positionRequests.add(request);
  }

  public MotorIOTalonFX usingControlRequest(PositionVoltage request) {
    addControlRequestPrivate(angle -> leader.setControl(request.withPosition(angle)));
    return this;
  }

  public MotorIOTalonFX usingControlRequest(MotionMagicVoltage request) {
    addControlRequestPrivate(angle -> leader.setControl(request.withPosition(angle)));
    return this;
  }

  public MotorIOTalonFX usingControlRequest(MotionMagicExpoVoltage request) {
    addControlRequestPrivate(angle -> leader.setControl(request.withPosition(angle)));
    return this;
  }

  public MotorIOTalonFX withCANcoder(EncoderIOCANcoder encoder) {
    tryUntilOk(
        5,
        () ->
            leader
                .getConfigurator()
                .apply(
                    new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                        .withFeedbackRemoteSensorID(encoder.getDeviceID())));
    return this;
  }

  @Override
  public int getNumFollowers() {
    return followers.length;
  }

  @Override
  public void close() {
    if (resetPosition != null) {
      resetPosition.stop();
      resetPosition.close();
    }
    leader.close();
    for (TalonFX follower : followers) {
      follower.close();
    }
  }
}
