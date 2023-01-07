package com.team7419.joystick;

import edu.wpi.first.wpilibj2.command.button.Button;

public class DoubleButton extends Button {
    private final Button button1;
    private final Button button2;

    public DoubleButton(Button button1, Button button2) {
        this.button1 = button1;
        this.button2 = button2;
    }

    @Override
    public boolean get() {
        return button1.get() && button2.get();
    }
}
