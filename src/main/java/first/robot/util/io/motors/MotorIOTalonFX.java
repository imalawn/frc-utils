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
import org.wpilib.system.Notifier;
import org.wpilib.units.measure.*;

public class MotorIOTalonFX implements AutoCloseable, RollerIO, PivotIO, LinearSystemIO {
  private final TalonFX leader;
  private final TalonFX[] followers;

  private VelocityVoltage velocityRequest;
  private PositionRequest[] positionRequests;
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

  @FunctionalInterface
  private interface PositionRequest {
    void apply(TalonFX motor, Angle angle);
  }

  @SuppressWarnings("resource")
  private MotorIOTalonFX(
      CANBus canbus,
      int id,
      TalonFXConfiguration config,
      int[] followerIds,
      MotorAlignmentValue[] followerAlignments,
      PositionRequest[] positionRequests,
      EncoderIOCANcoder encoder) {
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
    velocity.setUpdateFrequency(100.0);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, voltage, statorCurrent, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, followerTemps);
    leader.optimizeBusUtilization();
    ParentDevice.optimizeBusUtilizationForAll(followers);
    PhoenixUtil.registerSignals(canbus, velocity, voltage, statorCurrent, temp);
    PhoenixUtil.registerSignals(canbus, followerTemps);
    tryUntilOk(5, () -> leader.setPosition(0));
    // Set follower behavior
    if (followers.length != followerAlignments.length) {
      throw new IllegalArgumentException("Every follower motor must have an alignment value!");
    }
    for (int i = 0; i < followers.length; i++) {
      followers[i].setControl(new Follower(leader.getDeviceID(), followerAlignments[i]));
    }
    this.positionRequests = positionRequests;
    // Configure feedback
    if (encoder != null) {
      tryUntilOk(
          5,
          () ->
              leader
                  .getConfigurator()
                  .apply(
                      new FeedbackConfigs()
                          .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                          .withFeedbackRemoteSensorID(encoder.getDeviceID())));
    }
  }

  @Deprecated
  public MotorIOTalonFX(
      CANBus canbus,
      int id,
      TalonFXConfiguration config,
      int[] followerIds,
      MotorAlignmentValue[] followerAlignments) {
    this(canbus, id, config, followerIds, followerAlignments, new PositionRequest[0], null);
  }

  @Deprecated
  public MotorIOTalonFX(CANBus canbus, int id, TalonFXConfiguration config) {
    this(canbus, id, config, new int[0], new MotorAlignmentValue[0]);
  }

  @Deprecated
  public MotorIOTalonFX withControlRequest(PositionVoltage request) {
    positionRequests =
        new PositionRequest[] {(leader, angle) -> leader.setControl(request.withPosition(angle))};
    return this;
  }

  @Deprecated
  public MotorIOTalonFX withControlRequest(MotionMagicVoltage request) {
    positionRequests =
        new PositionRequest[] {(leader, angle) -> leader.setControl(request.withPosition(angle))};
    return this;
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
    inputs.velocityDegPerSec = velocity.getValue().in(DegreesPerSecond);
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
    positionRequests[0].apply(leader, angle);
  }

  @Override
  public void setPosition(int slot, Angle angle) {
    if (slot < 0 || slot >= positionRequests.length) return;
    positionRequests[slot].apply(leader, angle);
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
    angleResetVal = angle;
    resetPosition.startSingle(0);
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

  private void configurePositionControl() {
    if (resetPosition != null) return;
    if (positionRequests == null || positionRequests.length == 0) {
      PositionVoltage defaultRequest = new PositionVoltage(0);
      positionRequests =
          new PositionRequest[] {
            (leader, angle) -> leader.setControl(defaultRequest.withPosition(angle))
          };
    }
    resetPosition = new Notifier(() -> leader.setPosition(angleResetVal));
  }

  private void configureVelocityControl() {
    if (velocityRequest != null) return;
    velocityRequest = new VelocityVoltage(0);
    velocity.setUpdateFrequency(100.0);
    PhoenixUtil.registerSignals(leader.getNetwork(), velocity);
  }

  public static class Builder {
    private record FollowerMotor(int id, MotorAlignmentValue alignment) {}

    private final CANBus canbus;
    private final int id;
    private final TalonFXConfiguration config;
    private final ArrayList<FollowerMotor> followers = new ArrayList<>();
    private final ArrayList<PositionRequest> positionRequests = new ArrayList<>();
    private EncoderIOCANcoder encoder;

    public Builder(CANBus canbus, int id, TalonFXConfiguration config) {
      this.canbus = canbus;
      this.id = id;
      this.config = config;
    }

    public Builder addFollower(int id, MotorAlignmentValue alignment) {
      followers.add(new FollowerMotor(id, alignment));
      return this;
    }

    public Builder addControlRequest(PositionVoltage request) {
      positionRequests.add((leader, angle) -> leader.setControl(request.withPosition(angle)));
      return this;
    }

    public Builder addControlRequest(MotionMagicVoltage request) {
      positionRequests.add((leader, angle) -> leader.setControl(request.withPosition(angle)));
      return this;
    }

    public Builder addControlRequest(MotionMagicExpoVoltage request) {
      positionRequests.add((leader, angle) -> leader.setControl(request.withPosition(angle)));
      return this;
    }

    public Builder addCANCoder(EncoderIOCANcoder encoder) {
      this.encoder = encoder;
      return this;
    }

    public MotorIOTalonFX build() {
      int[] followerIds = new int[followers.size()];
      MotorAlignmentValue[] followerAlignments = new MotorAlignmentValue[followers.size()];
      for (int i = 0; i < followers.size(); i++) {
        followerIds[i] = followers.get(i).id;
        followerAlignments[i] = followers.get(i).alignment;
      }
      return new MotorIOTalonFX(
          canbus,
          id,
          config,
          followerIds,
          followerAlignments,
          positionRequests.toArray(new PositionRequest[0]),
          encoder);
    }
  }
}
