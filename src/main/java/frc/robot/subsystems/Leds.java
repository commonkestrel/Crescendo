package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LEDConstants;

public class Leds extends SubsystemBase {
    public static enum State {
        Solid,
        Flash,
        FastFlash,
        Fade,
    }

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private long m_previousNanos = 0;
    private State m_currentState = State.Solid;
    private Color m_currentColor = Color.kBlack;
    private boolean m_flashOn = true;

    private static Leds m_instance;
    
    public static Leds getInstance() {
        if (m_instance == null) {
            m_instance = new Leds(IOConstants.ledPort, IOConstants.ledLength);
        }

        return m_instance;
    }

    private Leds(int port, int length) {
        m_led = new AddressableLED(port);
        m_buffer = new AddressableLEDBuffer(length);

        m_led.setLength(m_buffer.getLength());
        m_led.setData(m_buffer);
        m_led.start();
    }

    public void fill(Color color) {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setLED(i, color);
        }
    }

    public void set(State state, Color color) {
        m_currentState = state;
        m_currentColor = color;
        m_previousNanos = System.nanoTime();
        m_flashOn = true;

        fill(color);
        update();
    }

    public void update() {
        m_led.setData(m_buffer);
    }

    public Color faded(Color base, double fade) {
        double red = base.red * fade;
        double green = base.green * fade;
        double blue = base.blue * fade;

        return new Color(red, green, blue);
    }

    public void invert() {
        fill(m_flashOn ? Color.kBlack : m_currentColor);
        m_flashOn = !m_flashOn;
        update();
    }

    @Override
    public void periodic() {
        switch (m_currentState) {
        case Solid:
            break;
        case Flash:
            if (System.nanoTime() - m_previousNanos > LEDConstants.flashLength) invert();
        case FastFlash:
            if (System.nanoTime() - m_previousNanos > LEDConstants.fastFlashLength) invert();
        case Fade:
            double fade = Math.cos(2.0 * Math.PI * ((double) (System.nanoTime() - m_previousNanos)) / LEDConstants.fadePeriod) / 2.0 + 0.5;
            fill(faded(m_currentColor, fade));
            update();
        }
    }
}
