// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package first.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;
import org.wpilib.system.Notifier;

public final class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Signals for synchronized refresh. */
  private static BaseStatusSignal[] drivebaseSignals = new BaseStatusSignal[0];

  private static BaseStatusSignal[] superstructureSignals = new BaseStatusSignal[0];

  /** Notifier loop for signal refresh */
  private static final Notifier signalThread = new Notifier(PhoenixUtil::waitForAll);

  /** Registers a set of signals for synchronized refresh. */
  public static void registerSignals(CANBus canbus, BaseStatusSignal... signals) {
    if (canbus.getName().equals("Drivebase")) {
      BaseStatusSignal[] newSignals =
          new BaseStatusSignal[drivebaseSignals.length + signals.length];
      System.arraycopy(drivebaseSignals, 0, newSignals, 0, drivebaseSignals.length);
      System.arraycopy(signals, 0, newSignals, drivebaseSignals.length, signals.length);
      drivebaseSignals = newSignals;
    } else {
      BaseStatusSignal[] newSignals =
          new BaseStatusSignal[superstructureSignals.length + signals.length];
      System.arraycopy(superstructureSignals, 0, newSignals, 0, superstructureSignals.length);
      System.arraycopy(signals, 0, newSignals, superstructureSignals.length, signals.length);
      superstructureSignals = newSignals;
    }
  }

  /** Refresh all registered signals. */
  public static void waitForAll() {
    if (drivebaseSignals.length > 0) {
      BaseStatusSignal.waitForAll(0.02, drivebaseSignals);
    }
    if (superstructureSignals.length > 0) {
      BaseStatusSignal.waitForAll(0.02, superstructureSignals);
    }
  }

  /** Start a thread for refreshing signals */
  public static void startTelemetry() {
    signalThread.startPeriodic(0.02);
  }
}
