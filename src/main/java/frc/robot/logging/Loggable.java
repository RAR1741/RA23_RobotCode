package frc.robot.logging;

public interface Loggable {
	/**
	* Logs all the headers provided by the Loggable object.
	*/
	public abstract void logHeaders(Logger logger);

	/**
	* Logs all the data provided by the Loggable object.
	*/
	public abstract void logData(Logger logger);
}