package first.robot.util;

import java.util.*;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.MatchState;

/**
 * This class contains methods that are used throughout the codebase and are not bound to one
 * subsystem or class.
 */
public final class RobotUtil {
  public record RumbleRequest(double left, double right, int priority, boolean isTriggerRumble)
      implements Comparable<RumbleRequest> {
    public RumbleRequest(double intensity, int priority) {
      this(intensity, intensity, priority, false);
    }

    public RumbleRequest(double left, double right, int priority) {
      this(left, right, priority, false);
    }

    @Override
    public int compareTo(RumbleRequest other) {
      return other.priority - this.priority;
    }
  }

  @Setter private static CommandNiDsXboxController driverController;
  @Setter private static CommandNiDsXboxController operatorController;
  private static final PriorityQueue<RumbleRequest> driverRumble = new PriorityQueue<>();
  private static final PriorityQueue<RumbleRequest> operatorRumble = new PriorityQueue<>();

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  public static boolean isRedAlliance() {
    var alliance = MatchState.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.RED;
  }

  /** Request driver controller rumble. */
  public static void requestDriverRumble(RumbleRequest request) {
    if (request == null) return;
    if (!driverRumble.contains(request)) {
      driverRumble.offer(request);
      updateDriverRumble();
    }
  }

  /** Stop driver controller rumble. */
  public static void stopDriverRumble(RumbleRequest request) {
    if (request == null) return;
    if (driverRumble.remove(request)) {
      updateDriverRumble();
    }
  }

  /** Request operator controller rumble. */
  public static void requestOperatorRumble(RumbleRequest request) {
    if (request == null) return;
    if (!operatorRumble.contains(request)) {
      operatorRumble.offer(request);
      updateOperatorRumble();
    }
  }

  /** Stop operator controller rumble. */
  public static void stopOperatorRumble(RumbleRequest request) {
    if (request == null) return;
    if (operatorRumble.remove(request)) {
      updateOperatorRumble();
    }
  }

  /** Update driver controller rumbles based on current queue head */
  private static void updateDriverRumble() {
    if (driverController == null) return;
    Logger.recordOutput("XboxController/DriverRumble", driverRumble.toArray(new RumbleRequest[0]));
    RumbleRequest active = driverRumble.peek();
    if (active != null) {
      if (active.isTriggerRumble) {
        driverController.setRumble(GenericHID.RumbleType.LEFT_TRIGGER_RUMBLE, active.left);
        driverController.setRumble(GenericHID.RumbleType.RIGHT_TRIGGER_RUMBLE, active.right);
      } else {
        driverController.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, active.left);
        driverController.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, active.right);
      }
    } else {
      driverController.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.0);
      driverController.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0.0);
      driverController.setRumble(GenericHID.RumbleType.LEFT_TRIGGER_RUMBLE, 0.0);
      driverController.setRumble(GenericHID.RumbleType.RIGHT_TRIGGER_RUMBLE, 0.0);
    }
  }

  /** Update operator controller rumbles based on current queue head */
  private static void updateOperatorRumble() {
    if (operatorController == null) return;
    Logger.recordOutput(
        "XboxController/OperatorRumble", operatorRumble.toArray(new RumbleRequest[0]));
    RumbleRequest active = operatorRumble.peek();
    if (active != null) {
      if (active.isTriggerRumble) {
        operatorController.setRumble(GenericHID.RumbleType.LEFT_TRIGGER_RUMBLE, active.left);
        operatorController.setRumble(GenericHID.RumbleType.RIGHT_TRIGGER_RUMBLE, active.right);
      } else {
        operatorController.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, active.left);
        operatorController.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, active.right);
      }
    } else {
      operatorController.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.0);
      operatorController.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0.0);
      operatorController.setRumble(GenericHID.RumbleType.LEFT_TRIGGER_RUMBLE, 0.0);
      operatorController.setRumble(GenericHID.RumbleType.RIGHT_TRIGGER_RUMBLE, 0.0);
    }
  }
}
