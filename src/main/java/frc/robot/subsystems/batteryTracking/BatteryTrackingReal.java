package frc.robot.subsystems.batteryTracking;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.ZonedDateTime;

public class BatteryTrackingReal implements BatteryTrackingIO {
  private static final String batteryIDFile =
      Constants.currentMode == Constants.Mode.REAL ? "/home/lvuser/batteryID.txt" : "batteryID.txt";
  private final NetworkTable table;

  private final IntegerPublisher epochLocalTimeMinPub;
  private final BooleanPublisher writePub;
  private final DoublePublisher usageSupplier;
  private final BooleanSubscriber failedSub;

  private IntegerSubscriber idSub;

  private boolean finished = false;

  public BatteryTrackingReal() {
    table = NetworkTableInstance.getDefault().getTable("BatteryTracking");
    epochLocalTimeMinPub = table.getIntegerTopic("epochLocalTimeMin").publish();
    writePub = table.getBooleanTopic("triggerWrite").publish();
    usageSupplier = table.getDoubleTopic("usageAH").publish();
    failedSub = table.getBooleanTopic("failed").subscribe(false);
    idSub = table.getIntegerTopic("batteryID").subscribe(-1);
  }

  @Override
  public void updateInputs(BatteryTrackingInputs inputs) {
    inputs.failed = failedSub.get();
    // Don't do anything if the system time is invalid
    if (!RobotController.isSystemTimeValid()) {
      return;
    }
    // Give the coprocessor the real time
    epochLocalTimeMinPub.set(ZonedDateTime.now().toEpochSecond() / (60));
    // Check what it read
    if (!finished && idSub.get() != -1) {
      inputs.batteryID = (int) idSub.get();
      try (StringSubscriber nameSub = table.getStringTopic("batteryName").subscribe("Unknown");
          IntegerSubscriber yearSub = table.getIntegerTopic("batteryYear").subscribe(-1);
          RawSubscriber logSub =
              table.getRawTopic("log").subscribe("BatteryTrackingLogEntry[]", new byte[0])) {
        inputs.batteryName = nameSub.get();
        inputs.batteryYear = (int) yearSub.get();
        inputs.serializedLog = logSub.get();
      }
      if (!inputs.batteryName.equals("Unknown") && inputs.batteryYear != -1) {
        finished = true;
        idSub.close();
        // Let the GC take it away
        idSub = null;

        File file = new File(batteryIDFile);
        if (file.exists()) {
          // Read previous battery name
          int previousBatteryID = -1;
          try {
            previousBatteryID = Integer.parseInt(Files.readString(Paths.get(batteryIDFile)));
          } catch (IOException e) {
            DriverStation.reportError("Failed to read battery ID file", e.getStackTrace());
          }
          if (previousBatteryID == inputs.batteryID) {
            // Same battery, set alert
            inputs.reusedBattery = true;
          } else {
            // New battery, delete file
            if (!file.delete()) {
              DriverStation.reportError("Failed to delete battery ID file", false);
            }
          }
        }

        // Write battery name if connected to FMS
        if (DriverStation.isFMSAttached()) {
          try {
            FileWriter fileWriter = new FileWriter(batteryIDFile);
            fileWriter.write(Integer.toString(inputs.batteryID));
            fileWriter.close();
          } catch (IOException e) {
            DriverStation.reportError("Failed to write battery ID file", e.getStackTrace());
          }
        }
      }
    }
  }

  @Override
  public void triggerWrite() {
    writePub.set(true);
  }

  @Override
  public void setUsageAH(double usageAH) {
    usageSupplier.set(usageAH);
  }
}
