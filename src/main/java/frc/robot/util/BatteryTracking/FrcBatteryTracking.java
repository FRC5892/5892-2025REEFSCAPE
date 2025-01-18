package frc.robot.util.BatteryTracking;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants;
import frc.robot.Robot;
import java.io.FileWriter;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;

public class FrcBatteryTracking {
  private static final String batteryFilePath = "/home/lvuser/battery-id.txt";
  private final Alert reusedBatteryAlert =
      new Alert("Battery has not been changed since the last match.", Alert.AlertType.kWarning);

  private boolean hasWritten = false;
  private boolean haveBatteryData = false;
  /** Previous Battery's ID 0 = not attempted -1 = failed */
  private final int previousBatteryID = 0;

  private boolean enabledLastCycle = false;
  private double batteryUsageAH = 0;
  private PowerDistribution powerDistribution = null;

  public FrcBatteryTracking(PowerDistribution powerDistribution) {
    Alert failedToReadAlert = new Alert("Failed to read battery data", Alert.AlertType.kWarning);
    if (Constants.currentMode != Constants.Mode.REAL) {
      return;
    }
    this.powerDistribution = powerDistribution;
    BatteryTracking.ERROR_HANDLER = new DriverStationLogger();
    BatteryTracking.initialRead();
    BatteryTracking.Battery insertedBattery = BatteryTracking.getInsertedBattery();
    if (insertedBattery == null) {
      failedToReadAlert.set(true);
      return;
    }
    haveBatteryData = true;
    Logger.recordOutput("Battery/Id", insertedBattery.getId());
    Logger.recordOutput("Battery/Name", insertedBattery.getName());
    Logger.recordOutput("Battery/Year", insertedBattery.getYear());
    Logger.recordOutput("Battery/InitialUsageAH", insertedBattery.getInitialUsageAH());
    Logger.recordOutput(
        "Battery/LastUsed", insertedBattery.getLog().get(0).getDateTime().toString());
    BatteryTracking.updateSync(batteryUsageAH);
    BatteryTracking.updateAutonomously(this::usageSupplierAH);
  }

  public double usageSupplierAH() {
    return batteryUsageAH;
  }

  public void periodic() {
    if (Constants.currentMode != Constants.Mode.REAL) {
      return;
    }
    if (enabledLastCycle && DriverStation.isDisabled()) {
      BatteryTracking.manualAsyncUpdate();
    }
    double current = powerDistribution.getTotalCurrent();
    batteryUsageAH += (current * (Robot.defaultPeriodSecs / (60 * 60)));
    if (haveBatteryData && DriverStation.isFMSAttached() && !hasWritten) {
      hasWritten = true;
      try {
        FileWriter fileWriter = new FileWriter(batteryFilePath);
        fileWriter.write(BatteryTracking.getInsertedBattery().getId());
        fileWriter.close();
      } catch (IOException e) {
        DriverStation.reportError(
            "Could not write battery file: " + e.getMessage(), e.getStackTrace());
      }
    }
    enabledLastCycle = DriverStation.isEnabled();
  }

  private static class DriverStationLogger implements BatteryTracking.ProgramSpecificErrorHandling {

    @Override
    public void consumeError(String message, Exception e) {
      DriverStation.reportError(message, e.getStackTrace());
    }

    @Override
    public void consumeError(String message) {
      DriverStation.reportError(message, true);
    }

    @Override
    public void consumeWarning(String message, Exception e) {
      DriverStation.reportWarning(message, e.getStackTrace());
    }

    @Override
    public void consumeWarning(String message) {
      DriverStation.reportWarning(message, false);
    }
  }
}
