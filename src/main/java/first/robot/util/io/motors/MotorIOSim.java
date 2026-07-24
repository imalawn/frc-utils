package first.robot.util.io.motors;

import java.util.Arrays;
import org.wpilib.math.controller.PIDController;

public class MotorIOSim implements MotorIO {
  private final int numFollowers;

  protected double appliedVoltage;
  protected final PIDController pid;
  protected boolean isClosedLoop;

  public MotorIOSim(double kP, double kD, int numFollowers) {
    this.numFollowers = numFollowers;
    pid = new PIDController(kP, 0.0, kD);
  }

  protected void updateMotorInputs(MotorIOInputs inputs) {
    inputs.connected = true;
    inputs.appliedVoltage = appliedVoltage;
    inputs.tempCelsius = 0.0;

    Arrays.fill(inputs.followerConnected, true);
    Arrays.fill(inputs.followerTempCelsius, 0.0);
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    appliedVoltage = volts;
  }

  @Override
  public void coast() {
    isClosedLoop = false;
    appliedVoltage = 0.0;
  }

  @Override
  public void brake() {
    isClosedLoop = false;
    appliedVoltage = 0.0;
  }

  @Override
  public int getNumFollowers() {
    return numFollowers;
  }
}
