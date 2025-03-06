package frc.robot.subsystems.batteryTracking;

import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.time.LocalDateTime;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;

public class BatteryTracking extends SubsystemBase {
  private final Timer writeTimer = new Timer();
  private final Alert readAlert =
      new Alert("Battery has not been read yet", Alert.AlertType.kWarning);
  private final Alert failedAlert = new Alert("Battery Tracking failed!", Alert.AlertType.kError);
  private final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.", Alert.AlertType.kWarning);
  private double batteryUsageAH = 0;
  private final DoubleSupplier currentSupplier;

  private final BatteryTrackingIO io;
  private final BatteryTrackingIO.BatteryTrackingInputs inputs =
      new BatteryTrackingIO.BatteryTrackingInputs();

  private LogEntry[] log;

  private boolean dataProcessed = false;

  public BatteryTracking(BatteryTrackingIO io, DoubleSupplier currentSupplier) {
    this.currentSupplier = currentSupplier;
    this.io = io;
    readAlert.set(true);
    writeTimer.start();
  }

  public BatteryTracking(BatteryTrackingIO io) {
    this(io, ConduitApi.getInstance()::getPDPTotalCurrent);
  }

  public Command writeCommand() {
    return new InstantCommand(this::write).ignoringDisable(true);
  }

  private void write() {
    io.setUsageAH(batteryUsageAH);
    io.triggerWrite();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("BatteryTracking", inputs);
    batteryUsageAH += (currentSupplier.getAsDouble() * (Robot.defaultPeriodSecs / (60 * 60)));

    if (inputs.batteryID != -1 && !dataProcessed) {
      dataProcessed = true;
      readAlert.set(false);
      log = StructBuffer.create(LogEntry.struct).readArray(inputs.serializedLog);
    }

    if (writeTimer.hasElapsed(180)) {
      writeTimer.reset();
      write();
    }

    // Only set the failed alert once, allowing it to move down the list
    if (inputs.failed && !failedAlert.get()) {
      failedAlert.set(true);
    }
    if (!inputs.failed) {
      failedAlert.set(false);
    }
    if (inputs.reusedBattery && !sameBatteryAlert.get()) {
      sameBatteryAlert.set(true);
    }
  }

  public int getBatteryID() {
    return inputs.batteryID;
  }

  public String getBatteryName() {
    return inputs.batteryName;
  }

  public LocalDateTime getLastBatteryUsage() {
    return log[0].getDateTime();
  }
}
