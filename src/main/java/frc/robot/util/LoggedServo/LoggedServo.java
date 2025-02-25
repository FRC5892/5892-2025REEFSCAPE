// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.LoggedServo;

/** Add your docs here. */
public abstract class LoggedServo {

  protected final int minPulse;
  protected final int maxPulse;
  protected final int halfPulse;
  protected final int centerPulse;

  public LoggedServo(int minPulse, int maxPulse) {
    this.minPulse = minPulse;
    this.maxPulse = maxPulse;
    this.halfPulse = calculateHalfPulse(minPulse, maxPulse);
    this.centerPulse = calculateCenterPulse(minPulse, maxPulse);
  }

  public abstract void setPulse(int pulse);

  public abstract void setEnabled(boolean enabled);

  public abstract void setPowered(boolean powered);

  public void setPercent(double percent) {
    final int pulse = minPulse + (int) (halfPulse * percent);
    this.setPulse(pulse);
  }

  public static int calculateHalfPulse(int minPulse, int maxPulse) {
    return ((maxPulse - minPulse) / 2);
  }

  public static int calculateCenterPulse(int minPulse, int maxPulse) {
    return calculateHalfPulse(minPulse, maxPulse) + minPulse;
  }
}
