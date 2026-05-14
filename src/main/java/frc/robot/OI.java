package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class OI /*GEVALD*/ {

    private final PlaystationControllerWrapper controller = new PlaystationControllerWrapper(0);
//    private final Joystick left = new Joystick(-1);
//    private final Joystick right = new PlaystationControllerWrapper(-1);

    private static final double lowSpeed = 0.7;
    private static final double highSpeed = 1;

    private double speedScale;
    private boolean isFieldRelative;

    public OI() {

        controller.getTriangleButton().onTrue(
          new InstantCommand(this::toggleSpeedScale)
        );

        controller.getCircleButton().onTrue(
            new InstantCommand(() -> isFieldRelative = !isFieldRelative
        ));
    }

    private void toggleSpeedScale() {
        if (speedScale == lowSpeed) {
            speedScale = highSpeed;
        } else {
            speedScale = lowSpeed;
        }
    }

    public boolean isFieldRelative() {
        return isFieldRelative;
    }

    public double getControllerLeftX() {
        return controller.getLeftX() * speedScale;
    }

    public double getControllerLeftY() {
        return controller.getLeftY() * speedScale;
    }

    public double getControllerRightX() {
        return controller.getRightX();
    }
}
