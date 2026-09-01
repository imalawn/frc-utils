package frc.robot.util.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashSet;
import java.util.Set;
import lombok.Getter;

public class SubsystemManager {
  private static SubsystemManager instance;
  @Getter private static boolean robotEnabled;

  /** Returns the singleton instance of the {@link SubsystemManager}. */
  public static SubsystemManager getInstance() {
    if (instance == null) {
      instance = new SubsystemManager();
    }
    return instance;
  }

  private final Set<Subsystem> subsystems = new HashSet<>();

  /**
   * Updates the {@link SubsystemManager#robotEnabled} flag to enabled and runs all subsystem enable
   * logic. Call this in your robot's {@code disabledExit()} method.
   */
  public void enable() {
    robotEnabled = true;
  }

  /**
   * Updates the {@link SubsystemManager#robotEnabled} flag to disabled and runs all subsystem
   * disable logic. Call this in your robot's {@code disabledInit()} method.
   */
  public void disable() {
    robotEnabled = false;
  }
}
