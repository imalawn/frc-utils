package first.robot.util.io.motors;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import first.robot.util.io.motors.elevator.LinearSystem;
import first.robot.util.io.motors.elevator.LinearSystemIO;
import first.robot.util.io.motors.pivot.Pivot;
import first.robot.util.io.motors.pivot.PivotIO;
import first.robot.util.io.motors.roller.Roller;
import first.robot.util.io.motors.roller.RollerIO;
import java.util.Arrays;
import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.Angle;

class MotorConfigureTest {
  @Test
  void rollerConfigurationDoesNotCauseErrors() {
    ConfigTrackingMotorIO io = new ConfigTrackingMotorIO();
    Roller roller = new Roller("TestRoller", io);

    assertDoesNotThrow(() -> roller.runVelocity(5.0));
    assertTrue(io.velocityControlCalled());
    assertDoesNotThrow(roller::getVelocity);
    assertDoesNotThrow(roller::getVelocityRPS);
    assertDoesNotThrow(() -> roller.runVoltage(3.0));
    assertDoesNotThrow(roller::stop);
    assertDoesNotThrow(roller::periodic);
  }

  @Test
  void pivotConfigurationDoesNotCauseErrors() {
    ConfigTrackingMotorIO io = new ConfigTrackingMotorIO();
    Pivot pivot = new Pivot("TestPivot", io);

    assertDoesNotThrow(() -> pivot.runPosition(Degrees.of(90.0)));
    assertTrue(io.positionControlCalled());
    assertDoesNotThrow(() -> pivot.resetPosition(Degrees.of(0.0)));
    assertTrue(io.resetPositionCalled());
    assertDoesNotThrow(pivot::getPosition);
    assertDoesNotThrow(pivot::getPositionDeg);
    assertDoesNotThrow(() -> pivot.runVoltage(3.0));
    assertDoesNotThrow(pivot::stop);
    assertDoesNotThrow(pivot::periodic);
  }

  @Test
  void linearSystemConfigurationDoesNotCauseErrors() {
    ConfigTrackingMotorIO io = new ConfigTrackingMotorIO();
    LinearSystem linearSystem = new LinearSystem("TestLinearSystem", io, 120.0, 0.05);

    assertDoesNotThrow(() -> linearSystem.runPosition(Degrees.of(45.0)));
    assertTrue(io.positionControlCalled());
    assertDoesNotThrow(() -> linearSystem.runPosition(Meters.of(0.05)));
    assertDoesNotThrow(() -> linearSystem.resetPosition(Degrees.of(0.0)));
    assertTrue(io.resetPositionCalled());
    assertDoesNotThrow(linearSystem::getPosition);
    assertDoesNotThrow(linearSystem::getPositionRad);
    assertDoesNotThrow(() -> linearSystem.runVoltage(3.0));
    assertDoesNotThrow(linearSystem::stop);
    assertDoesNotThrow(linearSystem::periodic);
  }

  @Test
  void misconfiguredRollerThrows() {
    ConfigTrackingMotorIO io = new MisconfiguringMotorIO();
    Roller roller = new Roller("TestRoller", io);

    assertThrows(NullPointerException.class, () -> roller.runVelocity(5.0));
  }

  @Test
  void misconfiguredPivotThrows() {
    ConfigTrackingMotorIO io = new MisconfiguringMotorIO();
    Pivot pivot = new Pivot("TestPivot", io);

    assertThrows(NullPointerException.class, () -> pivot.runPosition(Degrees.of(90.0)));
    assertThrows(NullPointerException.class, () -> pivot.resetPosition(Degrees.of(0.0)));
  }

  @Test
  void tempCriticalPreventsIoCalls() {
    ConfigTrackingMotorIO io = new ConfigTrackingMotorIO();
    Roller roller = new Roller("TestRoller", io);

    io.setTempCelsius(80.0);
    roller.periodic();
    assertTrue(roller.isTempCritical());

    assertDoesNotThrow(() -> roller.runVelocity(5.0));
    assertFalse(io.velocityControlCalled());
  }

  @Test
  void followerArraysAreSizedToNumFollowers() {
    ConfigTrackingMotorIO io = new ConfigTrackingMotorIO(2);
    Roller roller = new Roller("TestRoller", io);

    roller.periodic();
    assertEquals(2, io.getLastFollowerArrayLength());
  }

  @Test
  void mockDetectsMisconfiguration() {
    ConfigTrackingMotorIO io = new ConfigTrackingMotorIO();

    assertThrows(NullPointerException.class, () -> io.setVelocity(1.0));
    assertThrows(NullPointerException.class, () -> io.setPosition(Degrees.of(1.0)));
    assertThrows(NullPointerException.class, () -> io.resetPosition(Degrees.of(1.0)));
  }

  private static class MisconfiguringMotorIO extends ConfigTrackingMotorIO {
    @Override
    public void configure(boolean positionControl, boolean velocityControl) {
      super.configure(false, false);
    }
  }

  private static class ConfigTrackingMotorIO implements RollerIO, PivotIO, LinearSystemIO {
    private boolean positionConfigured;
    private boolean velocityConfigured;
    private boolean positionControlCalled;
    private boolean velocityControlCalled;
    private boolean resetPositionCalled;
    private double tempCelsius;
    private final int numFollowers;
    private int lastFollowerArrayLength = -1;

    ConfigTrackingMotorIO() {
      this(0);
    }

    ConfigTrackingMotorIO(int numFollowers) {
      this.numFollowers = numFollowers;
    }

    void setTempCelsius(double tempCelsius) {
      this.tempCelsius = tempCelsius;
    }

    boolean positionControlCalled() {
      return positionControlCalled;
    }

    boolean velocityControlCalled() {
      return velocityControlCalled;
    }

    boolean resetPositionCalled() {
      return resetPositionCalled;
    }

    int getLastFollowerArrayLength() {
      return lastFollowerArrayLength;
    }

    @Override
    public void configure(boolean positionControl, boolean velocityControl) {
      positionConfigured = positionControl;
      velocityConfigured = velocityControl;
    }

    @Override
    public int getNumFollowers() {
      return numFollowers;
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
      inputs.connected = true;
      inputs.tempCelsius = tempCelsius;
      updateFollowerInputs(inputs);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
      inputs.connected = true;
      inputs.tempCelsius = tempCelsius;
      inputs.positionDeg = 0.0;
      inputs.velocityDegPerSec = 0.0;
      updateFollowerInputs(inputs);
    }

    @Override
    public void updateInputs(LinearSystemIOInputs inputs) {
      inputs.connected = true;
      inputs.tempCelsius = tempCelsius;
      inputs.positionRad = 0.0;
      inputs.velocityRadPerSec = 0.0;
      updateFollowerInputs(inputs);
    }

    private void updateFollowerInputs(MotorIOInputs inputs) {
      lastFollowerArrayLength = inputs.followerConnected.length;
      Arrays.fill(inputs.followerConnected, true);
      Arrays.fill(inputs.followerTempCelsius, 0.0);
    }

    @Override
    public void setVelocity(double rps) {
      velocityControlCalled = true;
      if (!velocityConfigured) {
        throw new NullPointerException(
            "setVelocity called but velocity control was not configured");
      }
    }

    @Override
    public void setPosition(Angle angle) {
      positionControlCalled = true;
      if (!positionConfigured) {
        throw new NullPointerException(
            "setPosition called but position control was not configured");
      }
    }

    @Override
    public void setPosition(int slot, Angle angle) {
      setPosition(angle);
    }

    @Override
    public void resetPosition(Angle angle) {
      resetPositionCalled = true;
      if (!positionConfigured) {
        throw new NullPointerException(
            "resetPosition called but position control was not configured");
      }
    }
  }
}
