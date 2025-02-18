package frc.robot.subsystems.batteryTracking;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface BatteryTrackingIO {
  void updateInputs(BatteryTrackingInputs inputs);

  void triggerWrite();

  void setUsageAH(double usageAH);

  class BatteryTrackingInputs implements LoggableInputs {
    public boolean failed;
    public String batteryName;
    public int batteryID = -1;
    public int batteryYear;
    public byte[] serializedLog;
    public boolean reusedBattery;
    private boolean publishedStruct = false;

    @Override
    public void toLog(LogTable table) {
      table.put("failed", failed);
      table.put("batteryName", batteryName);
      table.put("batteryID", batteryID);
      table.put("batteryYear", batteryYear);
      if (!publishedStruct) {
        publishedStruct = true;
        try {
          Method addStructSchemaMethod =
              LogTable.class.getDeclaredMethod("addStructSchema", Struct.class, Set.class);
          addStructSchemaMethod.setAccessible(true);
          addStructSchemaMethod.invoke(table, LogEntry.struct, new HashSet<>());
        } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException e) {
          DriverStation.reportError("Failed to publish struct schema", e.getStackTrace());
        }
      }
      if (serializedLog != null) {
        table.put(
            "batteryLog", new LogTable.LogValue(serializedLog, "struct:BatteryTrackingLogEntry[]"));
      }
      table.put("reusedBattery", reusedBattery);
    }

    @Override
    public void fromLog(LogTable table) {
      failed = table.get("failed", false);
      batteryName = table.get("batteryName", "");
      batteryID = table.get("batteryID", -1);
      batteryYear = table.get("batteryYear", -1);
      serializedLog = table.get("batteryLog", new byte[0]);
      reusedBattery = table.get("reusedBattery", false);
    }
  }
}
