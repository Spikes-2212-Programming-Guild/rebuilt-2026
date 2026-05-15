package frc.robot.oi.devices;

import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.oi.Actions;

public class Controller implements InputDevice {

    private final PlaystationControllerWrapper controller;

    public Controller(int port) {
        controller = new PlaystationControllerWrapper(port);
    }

    @Override
    public void initActions(Actions actions) {
        controller.getTriangleButton().onTrue(new InstantCommand(actions.toggleSpeedScale()));
        controller.getCircleButton().onTrue(new InstantCommand(actions.toggleFieldRelative()));
        controller.getSquareButton().onTrue(new InstantCommand(actions.toggleSquareInputs()));
        controller.getCrossButton().onTrue(new InstantCommand(actions.toggleSlewRateLimiter()));
        controller.getLeftButton().onTrue(new InstantCommand(actions.toggleDeadband()));
    }

    @Override
    public double getX() {
        return controller.getLeftX();
    }

    @Override
    public double getY() {
        return controller.getLeftY();
    }

    @Override
    public double getZ() {
        return controller.getRightX();
    }

    @Override
    public String getName() {
        return controller.getName();
    }
}
