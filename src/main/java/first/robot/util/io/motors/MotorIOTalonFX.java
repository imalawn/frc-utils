package frc.robot.util.io.motors;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class MotorIOTalonFX implements MotorIO {
  protected final TalonFX leader;
  private final TalonFX[] followers;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final CoastOut coastRequest = new CoastOut();
  private final StaticBrake brakeRequest = new StaticBrake();

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final BaseStatusSignal[] followerTemps;

  public MotorIOTalonFX(CANBus canbus, int id, TalonFXConfiguration config) {
    this(canbus, id, new int[0], config, new MotorAlignmentValue[0]);
  }

  public MotorIOTalonFX(
      CANBus canbus,
      int id,
      int[] followerIds,
      TalonFXConfiguration config,
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
      follower.getConfigurator().apply(config);
    }
    // Create status signals
    voltage = leader.getMotorVoltage();
    supplyCurrent = leader.getSupplyCurrent();
    statorCurrent = leader.getStatorCurrent();
    temp = leader.getDeviceTemp();
    followerTemps = new BaseStatusSignal[followers.length];
    for (int i = 0; i < followerTemps.length; i++) {
      followerTemps[i] = followers[i].getDeviceTemp();
    }
    // Register status signals
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, voltage, supplyCurrent, statorCurrent, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, followerTemps);
    leader.optimizeBusUtilization();
    ParentDevice.optimizeBusUtilizationForAll(followers);
    PhoenixUtil.registerSignals(canbus, voltage, supplyCurrent, statorCurrent, temp);
    PhoenixUtil.registerSignals(canbus, followerTemps);
    leader.setPosition(0);
    // Set follower behavior
    for (int i = 0; i < followers.length; i++) {
      followers[i].setControl(new Follower(leader.getDeviceID(), followerAlignments[i]));
    }
  }

  protected void updateMotorInputs(MotorIOInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(voltage, supplyCurrent, statorCurrent, temp);
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();

    for (int i = 0; i < followerTemps.length; i++) {
      inputs.followerConnected[i] = followerTemps[i].getStatus().isOK();
      inputs.followerTempCelsius[i] = followerTemps[i].getValueAsDouble();
    }
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
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
  public int getNumFollowers() {
    return followers.length;
  }
}
