package org.usfirst.frc.team2473.framework;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.net.Socket;

/**
 * A convenience extension of the Socket class that has built in methods for
 * sending and receiving lines through the connection.
 * @author JosephM
 * @author mvoodarla
 * @author wang.patrick57@gmail.com
 */
public class UtilitySocket extends Socket {
	private BufferedReader reader;
	private PrintStream stream;
	
	/**
	 * Creates the socket and initializes the BufferedReader and PrintStream.
	 * @param host The IP of the server that the connection is occurring on.
	 * @param port The port the connection on this IP is occurring on.
	 * @throws IOException If an I/O error occurs when creating the socket.
	 */
	public UtilitySocket(String host, int port) throws IOException {
		super(host, port);
		System.out.println("after super in UtilitySocket constructor");
		reader = new BufferedReader(new InputStreamReader(getInputStream()));
		stream = new PrintStream(getOutputStream());
		System.out.println("Created Utility Socket");
	}
	
	/**
	 * Sends a string to the server end of the connection. This string automatically
	 * has a newline character appended to it.
	 * @param s the string to send
	 */
	public void sendLine(String s) {
		stream.print(s + "\n");
	}
	
	/**
	 * Obtains the string the server end of the connection sent, or null if nothing
	 * was sent. If the server end has sent multiple strings since the last time
	 * this method was called, they will be buffered and the oldest unread string
	 * will be returned.
	 * @return the string obtained
	 */
	public String getLine() {
		try {
			if(reader.ready()) {
				return reader.readLine();
			}
		} catch (IOException e) {
			System.out.println(e.getStackTrace());
		}
		
		return null;
	}
}
