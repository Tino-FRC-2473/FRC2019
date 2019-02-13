package org.usfirst.frc.team2473.framework;

/**
 *	Provides an interface to log debug statements. 
 */
public class Logger {
	
	/**
	 * The log levels.
	 */
	public enum LogLevel{
		
		/**
		 * A debug level log statement. This is useful for information that only needs to be available when actively 
		 * debugging an issue.
		 */
		Debug,
		
		/**
		 * An info level log statement. This is useful for general flow logging.
		 */
		Info,
		
		/**
		 * An error level log statement. This is useful for critical errors that occur during runtime.
		 */
		Error,
	};
	
	/**
	 * The current log level.
	 */
	public LogLevel logLevel = LogLevel.Error;
	
	private static Logger defaultLogger = null;
	
	/**
	 * The default log instance.
	 */
	public static Logger getInstance() {
		if (defaultLogger == null) {
			defaultLogger = new Logger();
		}
		return defaultLogger;
	}
	
	/**
	 * Logs the specified string at an error level.
	 *
	 * @param	string	The string to log. 
	 */
	public void logError( String string ){
		log( LogLevel.Error, string);
	}
	
	/**
	 * Logs the specified string at an info level.
	 *
	 * @param	string	The string to log. 
	 */
	public void logInfo( String string ){
		log( LogLevel.Info, string);
	}
	
	/**
	 * Logs the specified string at an debug level.
	 *
	 * @param	string	The string to log. 
	 */
	public void logDebug( String string ){
		log( LogLevel.Debug, string);
	}
	
	/**
	 * Logs the specified string at the specified level.
	 *
	 * @param	level	The log level.
	 * @param	string	The string to log. 
	 */
	public void log(LogLevel level, String string) {
		
		//	Only log messages at or above the current log level
		
		if ( level.ordinal() >= logLevel.ordinal() ) {
			final Throwable t = new Throwable();
			final StackTraceElement methodCaller = t.getStackTrace()[1];
			System.out.println(methodCaller.getClassName() + " " + methodCaller.getMethodName() + " " + "[" + methodCaller.getLineNumber() + "]: " + string);
		}
	}
}