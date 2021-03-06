package org.usfirst.frc.team2473.framework;

public abstract class State {
	private String name;

	public State(String name) {
		this.name = name;
	}

	public abstract void init();

	@Override
	public String toString() {
		return name;
	}

	public abstract State handleEvent(Enum event);
}