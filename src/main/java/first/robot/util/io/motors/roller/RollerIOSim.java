package first.robot.util.io.motors.roller;

import first.robot.util.io.motors.MotorIOSim;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.FlywheelSim;

public class RollerIOSim extends MotorIOSim implements RollerIO {
  private final FlywheelSim sim;

  public RollerIOSim(
      DCMotor motorModel,
      RotationalMechanismConstraints constraints,
      double kP,
      double kD,
      int numFollowers) {
    super(kP, kD, numFollowers);
    sim =
        new FlywheelSim(
            Models.flywheelFromPhysicalConstants(
                motorModel, constraints.moi(), constraints.reduction()),
            motorModel);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    if (isClosedLoop) {
      appliedVoltage =
          Math.clamp(
              pid.calculate(Units.radiansToRotations(sim.getAngularVelocity())), -12.0, 12.0);
    }
    updateMotorInputs(inputs);
    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);
    inputs.velocityRPS = Units.radiansToRotations(sim.getAngularVelocity());
    inputs.statorCurrentAmps = sim.getCurrentDraw();
  }

  @Override
  public void setVelocity(double rps) {
    pid.setSetpoint(rps);
    isClosedLoop = true;
  }
}
