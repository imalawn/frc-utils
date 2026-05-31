package frc.robot.util.io.motors.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.io.motors.MotorIOSim;

public class PivotIOSim extends MotorIOSim implements PivotIO {
  private final SingleJointedArmSim sim;

  public PivotIOSim(
      DCMotor motorModel,
      MechanismConstraints constraints,
      double kP,
      double kD,
      int numFollowers) {
    super(kP, kD, numFollowers);
    sim =
        new SingleJointedArmSim(
            motorModel,
            constraints.reduction(),
            constraints.moi(),
            constraints.radiusMeters(),
            constraints.minAngleRads(),
            constraints.maxAngleRads(),
            true,
            constraints.startingAngleRads());
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (isClosedLoop) {
      appliedVoltage =
          MathUtil.clamp(pid.calculate(Units.radiansToRotations(sim.getAngleRads())), -12.0, 12.0);
    }
    updateMotorInputs(inputs);
    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);
    inputs.positionDeg = Units.radiansToDegrees(sim.getAngleRads());
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = appliedVoltage / 12.0 * inputs.statorCurrentAmps;
  }

  @Override
  public void setPosition(double deg) {
    pid.setSetpoint(Units.degreesToRotations(deg));
    isClosedLoop = true;
  }
}
