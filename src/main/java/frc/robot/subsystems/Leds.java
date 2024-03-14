package frc.robot.subsystems;

import java.util.Optional;

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

    private Optional<Short> m_flashPoint = Optional.empty();
    private Color m_flashColor;
    private long m_lastFlash;

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
        
        if (m_flashPoint.isEmpty()) {
            fill(color);
            update();
        }
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

    public void flash(Color color) {
        m_flashColor = color;
        m_flashPoint = Optional.of((short) 0);
        m_lastFlash = System.nanoTime();
        fill(m_flashColor);
        update();
    }

    private void invert() {
        fill(m_flashOn ? Color.kBlack : m_currentColor);
        m_flashOn = !m_flashOn;
        update();
    }

    @Override
    public void periodic() {
        if (m_flashPoint.isPresent()) {
            if (System.nanoTime() - m_lastFlash >= 100000000) {
                m_lastFlash = System.nanoTime();
                Short flashPoint = m_flashPoint.get();
                m_flashPoint = Optional.of((short) (flashPoint + 1));

                System.out.printf("Flash Point: %d%n", flashPoint);

                if (flashPoint / 2 == 5) {
                    m_flashPoint = Optional.empty();
                    fill(m_currentColor);
                    m_flashOn = true;
                    m_previousNanos = System.nanoTime();
                    return;
                }
                if (flashPoint % 2 == 0) {
                    fill(m_flashColor);
                    update();
                } else {
                    fill(Color.kBlack);
                    update();
                }
            }
        } else {
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
}