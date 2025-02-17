package frc.robot.subsystems.batteryTracking;

public class BatteryTrackingNoOpp implements BatteryTrackingIO {
  @Override
  public void updateInputs(BatteryTrackingInputs inputs) {}

  @Override
  public void triggerWrite() {}

  @Override
  public void setUsageAH(double usageAH) {}
}
