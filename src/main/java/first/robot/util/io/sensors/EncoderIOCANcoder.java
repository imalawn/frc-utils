package first.robot.util.io.sensors;

import static first.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import first.robot.util.PhoenixUtil;
import org.wpilib.units.measure.Angle;

public class EncoderIOCANcoder implements EncoderIO {
  private final CANcoder cancoder;

  private final StatusSignal<Angle> position;

  public EncoderIOCANcoder(CANBus canbus, int id, CANcoderConfiguration config) {
    cancoder = new CANcoder(id, canbus);
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(config));
    position = cancoder.getAbsolutePosition();
    position.setUpdateFrequency(100.0);
    PhoenixUtil.registerSignals(canbus, position);
  }

  @Override
  public void updateInputs(EncoderIOInputs inputs) {
    inputs.connected = position.getStatus().isOK();
    inputs.absolutePosition = position.getValue();
  }

  public int getDeviceID() {
    return cancoder.getDeviceID();
  }
}
