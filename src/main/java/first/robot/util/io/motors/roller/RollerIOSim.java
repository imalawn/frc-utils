package frc.robot.util.io.motors.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.io.motors.MotorIOSim;

public class RollerIOSim extends MotorIOSim implements RollerIO {
  private final DCMotorSim sim;

  public RollerIOSim(
      DCMotor motorModel,
      MechanismConstraints constraints,
      double kP,
      double kD,
      int numFollowers) {
    super(kP, kD, numFollowers);
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motorModel, constraints.moi(), constraints.reduction()),
            motorModel);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    if (isClosedLoop) {
      appliedVoltage =
          MathUtil.clamp(
              pid.calculate(sim.getAngularVelocity().in(RotationsPerSecond)), -12.0, 12.0);
    }
    updateMotorInputs(inputs);
    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);
    inputs.velocityRPS = sim.getAngularVelocity().in(RotationsPerSecond);
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = appliedVoltage / 12.0 * inputs.statorCurrentAmps;
  }

  @Override
  public void setVelocity(double rps) {
    pid.setSetpoint(rps);
    isClosedLoop = true;
  }
}
