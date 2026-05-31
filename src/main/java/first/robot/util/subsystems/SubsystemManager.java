package first.robot.util.subsystems;

import java.util.HashSet;
import java.util.Set;
import lombok.Getter;

public class SubsystemManager {
  private static SubsystemManager instance;
  @Getter private static boolean robotEnabled;

  public static synchronized SubsystemManager getInstance() {
    if (instance == null) {
      instance = new SubsystemManager();
    }
    return instance;
  }

  private final Set<Subsystem> subsystems = new HashSet<>();

  public void registerSubsystem(Subsystem... subsystems) {
    for (Subsystem subsystem : subsystems) {
      if (subsystem == null) {
        continue;
      }
      if (this.subsystems.contains(subsystem)) {
        continue;
      }
      this.subsystems.add(subsystem);
    }
  }

  public void enable() {
    robotEnabled = true;
    subsystems.forEach(Subsystem::enable);
  }

  public void disable() {
    robotEnabled = false;
    subsystems.forEach(Subsystem::disable);
  }
}
