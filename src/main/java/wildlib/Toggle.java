package wildlib;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Utility class for people too lazy to just work with booleans
 */
public class Toggle implements BooleanSupplier, BooleanConsumer {
    private boolean m_state;

    public Toggle(boolean initialState) {
        m_state = initialState;
    }

    public Command setTrue() {
        return Commands.runOnce(() -> m_state = true);
    }

    public Command setFalse() {
        return Commands.runOnce(() -> m_state = false);
    }

    public Command invert() {
        return Commands.runOnce(() -> m_state = !m_state);
    }

    public boolean getAsBoolean() {
        return m_state;
    }   

    public void accept(boolean value) {
        m_state = value;
    }
}
