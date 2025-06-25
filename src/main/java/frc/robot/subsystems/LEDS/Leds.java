package frc.robot.subsystems.LEDS;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    private final int START = 7;
    private final int LEFT_END = 49+START;
    private final int RIGHT_START = LEFT_END +1;
    private final int RIGHT_END = RIGHT_START + 48; 
    private final CANdle candle = new CANdle(41);
    private final RGBWColor ORANGE = new RGBWColor( 255,  64, 0 );
    private final ColorFlowAnimation flowAnimation = new ColorFlowAnimation(START, RIGHT_END);


  public Leds() {
      candle.getConfigurator().apply(new LEDConfigs().withStripType(StripTypeValue.GRB));
      candle.getConfigurator().apply(new CANdleFeaturesConfigs().withVBatOutputMode(VBatOutputModeValue.Off));
      candle.setControl(flowAnimation.withLEDStartIndex(START).withLEDEndIndex(LEFT_END).withColor(ORANGE ).withSlot(0));
      candle.setControl(flowAnimation.withLEDStartIndex(RIGHT_START).withLEDEndIndex(RIGHT_END).withColor(ORANGE ).withSlot(1));

      

  }

  @Override
  public void periodic() {
  }
}
