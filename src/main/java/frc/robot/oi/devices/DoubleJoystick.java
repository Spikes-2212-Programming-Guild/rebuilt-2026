package frc.robot.oi.devices;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.Actions;

public class DoubleJoystick implements InputDevice {

    private final Joystick left;
    private final Joystick right;

    public DoubleJoystick(int leftPort, int rightPort) {
        left = new Joystick(leftPort);
        right = new Joystick(rightPort);
    }

    @Override
    public void initActions(Actions actions) {
        new JoystickButton(left, 1).onTrue(new InstantCommand(actions.toggleSpeedScale()));
        new JoystickButton(left, 2).onTrue(new InstantCommand(actions.toggleFieldRelative()));
        new JoystickButton(left, 3).onTrue(new InstantCommand(actions.toggleSquareInputs()));
        new JoystickButton(left, 4).onTrue(new InstantCommand(actions.toggleSlewRateLimiter()));
        new JoystickButton(left, 5).onTrue(new InstantCommand(actions.toggleDeadband()));
    }

    @Override
    public double getX() {
        return left.getX();
    }

    @Override
    public double getY() {
        return left.getY();
    }

    @Override
    public double getZ() {
        return right.getX();
    }

    @Override
    public String getName() {
        return left.getName() + " / " + right.getName();
    }
}
