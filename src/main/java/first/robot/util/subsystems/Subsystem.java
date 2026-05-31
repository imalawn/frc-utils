package first.robot.util.subsystems;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Scheduler;

import first.robot.util.commands.Commands;

public abstract class Subsystem extends Mechanism {
  @SuppressWarnings("this-escape")
  protected Subsystem() {
    super();
    SubsystemManager.getInstance().registerSubsystem(this);
    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public void enable() {}

  public void disable() {}

  public void periodic() {}

  /**
   * Constructs a command that runs an action once and another action when the command is
   * interrupted. Requires this mechanism.
   *
   * @param start the action to run on start
   * @param end the action to run on interrupt
   * @return the command
   */
  public NeedsNameBuilderStage startEnd(Runnable start, Runnable end) {
    return Commands.startEnd(start, end, this);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted, and then runs a
   * second action. Requires this mechanism.
   *
   * @param run the action to run every iteration
   * @param end the action to run on interrupt
   * @return the command
   */
  public NeedsNameBuilderStage runEnd(Runnable run, Runnable end) {
    return Commands.runEnd(run, end, this);
  }

  /**
   * Constructs a command that runs an action once, and then runs an action every iteration until
   * interrupted. Requires this mechanism.
   *
   * @param start the action to run on start
   * @param run the action to run every iteration
   * @return the command
   */
  public NeedsNameBuilderStage startRun(Runnable start, Runnable run) {
    return Commands.startRun(start, run, this);
  }
}
