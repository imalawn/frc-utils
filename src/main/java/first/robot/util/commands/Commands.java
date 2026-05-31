package first.robot.util.commands;

import java.util.Arrays;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;

/**
 * A class that contains legacy command factory methods implemented through the modern Commands v3
 * framework.
 */
public final class Commands {
  /**
   * Constructs a command that does nothing, finishing immediately.
   *
   * @return the command
   */
  public static Command none() {
    return Command.noRequirements(co -> {}).named("Empty Command");
  }

  /**
   * Constructs a command without requirements that runs an action once and finishes.
   *
   * @param action the action to run
   * @return the command
   */
  public static NeedsNameBuilderStage runOnce(Runnable action) {
    return Command.noRequirements(co -> action.run());
  }

  /**
   * Constructs a command that runs an action once and finishes.
   *
   * @param action the action to run
   * @param requirements subsystems the action requires
   * @return the command
   */
  public static NeedsNameBuilderStage runOnce(Runnable action, Mechanism... requirements) {
    return Command.requiring(Arrays.asList(requirements)).executing(co -> action.run());
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted.
   *
   * @param action the action to run
   * @param requirements subsystems the action requires
   * @return the command
   */
  public static NeedsNameBuilderStage run(Runnable action, Mechanism... requirements) {
    return Command.requiring(Arrays.asList(requirements))
        .executing(
            co -> {
              while (true) {
                action.run();
                co.yield();
              }
            });
  }

  /**
   * Constructs a command that runs an action once and another action when the command is
   * interrupted.
   *
   * @param start the action to run on start
   * @param end the action to run on interrupt
   * @param requirements subsystems the action requires
   * @return the command
   */
  public static NeedsNameBuilderStage startEnd(
      Runnable start, Runnable end, Mechanism... requirements) {
    return Command.requiring(Arrays.asList(requirements))
        .executing(
            co -> {
              start.run();
              co.park();
            })
        .whenCanceled(end);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted, and then runs a
   * second action.
   *
   * @param run the action to run every iteration
   * @param end the action to run on interrupt
   * @param requirements subsystems the action requires
   * @return the command
   */
  public static NeedsNameBuilderStage runEnd(
      Runnable run, Runnable end, Mechanism... requirements) {
    return Command.requiring(Arrays.asList(requirements))
        .executing(
            co -> {
              while (true) {
                run.run();
                co.yield();
              }
            })
        .whenCanceled(end);
  }

  /**
   * Constructs a command that runs an action once, and then runs an action every iteration until
   * interrupted.
   *
   * @param start the action to run on start
   * @param run the action to run every iteration
   * @param requirements subsystems the action requires
   * @return the command
   */
  public static NeedsNameBuilderStage startRun(
      Runnable start, Runnable run, Mechanism... requirements) {
    return Command.requiring(Arrays.asList(requirements))
        .executing(
            co -> {
              start.run();
              while (true) {
                run.run();
                co.yield();
              }
            });
  }
}
