package frc.robot.util.io.motors.roller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.io.motors.MotorIOTalonFX;

public class RollerIOTalonFX extends MotorIOTalonFX implements RollerIO {
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> velocity;

  public RollerIOTalonFX(CANBus canbus, int id, TalonFXConfiguration config) {
    this(canbus, id, new int[0], config, new MotorAlignmentValue[0]);
  }

  public RollerIOTalonFX(
      CANBus canbus,
      int id,
      int[] followerIds,
      TalonFXConfiguration config,
      MotorAlignmentValue[] followerAlignments) {
    super(canbus, id, followerIds, config, followerAlignments);
    velocity = leader.getVelocity();
    velocity.setUpdateFrequency(100.0);
    PhoenixUtil.registerSignals(canbus, velocity);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocityRPS = velocity.getValueAsDouble();
    updateMotorInputs(inputs);
  }

  @Override
  public void setVelocity(double rps) {
    leader.setControl(velocityRequest.withVelocity(rps));
  }
}
