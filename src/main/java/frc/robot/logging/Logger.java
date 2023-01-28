package frc.robot.logging;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.Files;
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.IOException;
import java.util.Calendar;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.Map;

public class Logger {
	private BufferedWriter logger;
	public Map<String, String> logData;
	private ArrayList<Loggable> loggables;
	private String logName;

	public Logger() {
		logData = new HashMap<String, String>();
		loggables = new ArrayList<Loggable>();
	}

	/**
	* Creates a new log file.
	*
	* @throws IOException
	*/
	public void createLog() throws IOException {
        Calendar calendar = Calendar.getInstance();
        String dir = (System.getProperty("os.name").equals("Linux")) ? "./src/main/java/frc/robot/logging/logs/" : "\\home\\lvuser\\logs\\";
		Path path = Paths.get(
            dir + calendar.get(Calendar.YEAR) + "-" +
            (calendar.get(Calendar.MONTH) + 1) + "-" +
            calendar.get(Calendar.DAY_OF_MONTH) + "-" +
            calendar.get(Calendar.HOUR_OF_DAY) + "-" +
            calendar.get(Calendar.MINUTE) + "-" +
            calendar.get(Calendar.SECOND) + ".csv"
        );
		
		logName = Files.createFile(path).toString();
		logger = new BufferedWriter(new FileWriter(logName, true));
	}

	/**
	* Creates a header that data can be attributed to.
	*
	* @param header The name of the header being created.
	*/
	public void addHeader(String header) {
		if (logData.containsKey(header)) {
			System.err.println("The header \"" + header + "\" already exists.");
		}
		else {
			logData.put(header, null);
		}
	}

	/**
	* Adds data under the specified header.
	*
	* @param header The header that the data will be attributed to.
	* @param data The data being logged.
	*/
	public void addData(String header, Object data) {
		if (!logData.containsKey(header)) {
			System.err.println("The header \"" + header + "\" does not exist.");
		}
		else {
			logData.put(header, data.toString());
		}
	}

	/**
	* Writes the created headers to the log file.
	*
	* @throws IOException
	*/
	public void writeHeaders() throws IOException {
		for (String header: logData.keySet()) {
			logger.write(header + ",");
		}

		logger.newLine();
		logger.close();
		logger = new BufferedWriter(new FileWriter(logName, true));
	}

	/**
	* Writes the accumulated data to the log file.
	*
	* @throws IOException
	*/
	public void writeData(String stage) throws IOException {
        if (stage == null) {
            for (Map.Entry<String, String> data: logData.entrySet()) {
                logger.write(data.getValue() + ",");
                logData.put(data.getKey(), null);
            }
        }
        else {
            String output = "========================================= Current Stage: " + stage + " =========================================";
            
            logger.newLine();
            for (int i = 0; i < output.length(); i++) {
                logger.write("=");
            }
            logger.newLine(); logger.write(output); logger.newLine();
            for (int i = 0; i < output.length(); i++) {
                logger.write("=");
            }
            logger.newLine();
        }

		logger.newLine();
		logger.close();
		logger = new BufferedWriter(new FileWriter(logName, true));
	}

	/**
	* Creates a new Loggable object.
	*
	* @param loggable The Loggable object to be registered.
	*/
	public void addLoggable(Loggable loggable) {
		loggables.add(loggable);
	}

	/**
	* Goes through every registered Loggable and calls the "logHeaders()" method.
	*/
	public void collectHeaders() {
		for (Loggable loggable: loggables) {
			loggable.logHeaders(this);
		}
	}

	/**
	* Goes through every registered Loggable and calls the "logData()" method.
	*/
	public void collectData() {
		for (Loggable loggable: loggables) {
			loggable.logData(this);
		}
	}
}