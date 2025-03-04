package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED mLed;
    private final AddressableLEDBuffer ledBuffer;

    public LedSubsystem() {
        mLed = new AddressableLED(LedConstants.LED_PORT);
        mLed.setLength(LedConstants.LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);
        mLed.setData(ledBuffer);
    }

    public void changeColor(int[] color) {
        for (int i = 0; i < LedConstants.LED_LENGTH; i++) {
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
        mLed.setData(ledBuffer);
        mLed.start();
    }

    public void stopLED() {
        mLed.stop();
    }
}
