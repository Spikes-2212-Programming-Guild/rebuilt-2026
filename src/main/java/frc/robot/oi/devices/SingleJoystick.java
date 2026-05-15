package frc.robot.oi.devices;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.Actions;

public class SingleJoystick implements InputDevice {

    private final Joystick joystick;

    public SingleJoystick(int port) {
        joystick = new Joystick(port);
    }

    @Override
    public void bindActions(Actions actions) {
        new JoystickButton(joystick, 1).onTrue(new InstantCommand(actions.toggleSpeedScale()));
        new JoystickButton(joystick, 2).onTrue(new InstantCommand(actions.toggleFieldRelative()));
        new JoystickButton(joystick, 3).onTrue(new InstantCommand(actions.toggleSquareInputs()));
        new JoystickButton(joystick, 5).onTrue(new InstantCommand(actions.toggleDeadband()));
        new JoystickButton(joystick, 6).onTrue(new InstantCommand(actions.toggleSlewRateLimiter()));
    }

    @Override
    public double getX() {
        return joystick.getX();
    }

    @Override
    public double getY() {
        return joystick.getY();
    }

    @Override
    public double getZ() {
        return joystick.getTwist();
    }

    @Override
    public String getName() {
        return joystick.getName();
    }
}
