// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableMeasure<M extends Measure<U>, U extends Unit> implements Supplier<M> {
  private static final String tableKey = "/Tuning";

  private final M defaultValue;
  private final LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, M> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableMeasure(String dashboardKey, M defaultValue) {
    String key = tableKey + "/" + dashboardKey + " " + defaultValue.unit().symbol();
    this.defaultValue = defaultValue;
    if (Constants.tuningMode) {
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue.in(defaultValue.unit()));
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  @SuppressWarnings("unchecked")
  public M get() {
    if (Constants.tuningMode) {
      U unit = defaultValue.unit();
      return (M) unit.of(dashboardNumber.get());
    } else {
      return defaultValue;
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    if (!Constants.tuningMode) return false;
    M currentValue = get();
    M lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  public M getAsDouble() {
    return get();
  }
}
