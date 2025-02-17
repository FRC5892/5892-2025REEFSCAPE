package frc.robot.subsystems.batteryTracking;

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

    @Override
    public void toLog(LogTable table) {
      table.put("failed", failed);
      table.put("batteryName", batteryName);
      table.put("batteryID", batteryID);
      table.put("batteryYear", batteryYear);
      table.put(
          "batteryLog",
          serializedLog /*new LogTable.LogValue(serializedLog, "BatteryTrackingLogEntry[]")*/);
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
