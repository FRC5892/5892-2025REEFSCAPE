package frc.robot.util.LoggedDIO;

public class NoOppDio extends LoggedDIO {
  public NoOppDio(String name) {
    super(name);
  }

  @Override
  protected void updateInputs(DIOInputsAutoLogged inputs) {}
}
