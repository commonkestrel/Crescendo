package wildlib;

/**
 * Throw to indicate that the requested operation is not yet implemented.
 * This exception supplements {@link UnsupportedOperationException}
 * by providing a more semantically rich description of the problem.
 * 
 * This can act as an exception based TODO tag.
 * 
 * @author Jett Bergthold
 */
public class NotYetImplemented extends UnsupportedOperationException {
    /** The default error message that is displayed when none is provided */
    private static final String DEFAULT_MESSAGE = "not yet implemented";

    /** Constructs a NotYetImplemented exception with the default message. */
    public NotYetImplemented() {
        this(DEFAULT_MESSAGE);
    }

    /**
     * Constructs a NotYetImplemented exception with the given message.
     * 
     * @param message description of the exception
     */
    public NotYetImplemented(final String message) {
        super(message);
    }

    /**
     * Constructs a NotYetImplemented exception with the given message and cause.
     * 
     * @param message description of the exception
     * @param cause cause of the exception
     */
    public NotYetImplemented(final String message, final Throwable cause) {
        super(message, cause);
    }

    /**
     * Constructs a NotYetImplemented exception with the default message and given cause.
     * 
     * @param cause cause of the exception
     */
    public NotYetImplemented(final Throwable cause) {
        this(DEFAULT_MESSAGE, cause);
    }
}
