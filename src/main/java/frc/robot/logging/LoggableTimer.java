package frc.robot.logging;

import edu.wpi.first.wpilibj.Timer;

public class LoggableTimer extends Timer implements Loggable {
    @Override
    public void logHeaders(Logger logger) {
        logger.addHeader("Timer");
    }

    @Override
    public void logData(Logger logger) {
        logger.addData("Timer", this.get());
    }
}
