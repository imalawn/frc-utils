package first.robot.util.io.motors.pivot;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import first.robot.util.io.motors.MotorIOSim;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.units.measure.Angle;

public class PivotIOSim extends MotorIOSim implements PivotIO {
  private final SingleJointedArmSim sim;

  public PivotIOSim(
      DCMotor motorModel,
      RotationalMechanismConstraints constraints,
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
          Math.clamp(pid.calculate(Units.radiansToRotations(sim.getAngle())), -12.0, 12.0);
    }
    updateMotorInputs(inputs);
    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);
    inputs.positionDeg = Units.radiansToDegrees(sim.getAngle());
    inputs.statorCurrentAmps = sim.getCurrentDraw();
  }

  @Override
  public void setPosition(Angle angle) {
    pid.setSetpoint(angle.in(Rotations));
    isClosedLoop = true;
  }

  @Override
  public void resetPosition(Angle angle) {
    sim.setState(angle.in(Radians), 0.0);
  }
}
