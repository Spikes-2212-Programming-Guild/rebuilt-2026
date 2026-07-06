package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.advancedcommands.*;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.commands.intake.SpinCollection;
import frc.robot.commands.swerve.ModuleRotateWithPID;
import frc.robot.commands.swerve.RotateAccordingToGyro;
import frc.robot.subsystems.swerve.Drivetrain;

public class OI /*GEVALD*/ {

    private final Joystick driverRight = new Joystick(0);
    private final Joystick driverLeft = new Joystick(1);
    private final PlaystationControllerWrapper navigator = new PlaystationControllerWrapper(2);

    public OI() {
        configureDriver();
        configureNavigator();
    }

    private void configureDriver() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        new JoystickButton(driverRight, 1).onTrue(new InstantCommand(drivetrain::resetFieldRelativity));
        new JoystickButton(driverRight, 2).onTrue(
                new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        new JoystickButton(driverRight, 3).whileTrue(
                new RotateAccordingToGyro(270.0, true));
        new JoystickButton(driverRight, 4).whileTrue(
                new RotateAccordingToGyro(90.0, true));

        new JoystickButton(driverLeft, 4).onTrue(new RotateToTag(2));
        new JoystickButton(driverLeft, 3).onTrue(new RotateToTag(-2));
        new JoystickButton(driverLeft, 2).whileTrue(new ModuleRotateWithPID(drivetrain,
                45.0, 135.0, 135.0, 45.0));
    }

    private void configureNavigator() {
        navigator.getR2Button().whileTrue(new SpinCollection());
        navigator.getTriangleButton().onTrue(new UpWithJumpies());
        navigator.getUpButton().whileTrue(new MoveCollection(-0.4));
        navigator.getDownButton().whileTrue(new MoveCollection(0.4));
        navigator.getCrossButton().onTrue(new MoveDown());

        navigator.getL1Button().whileTrue(new ShootToHub());
        navigator.getL2Button().whileTrue(new Pass());
        navigator.getCircleButton().whileTrue(new ShootFromTrench());

        navigator.getLeftButton().onTrue(new RotateToTag(2));
        navigator.getRightButton().onTrue(new RotateToTag(-2));

        navigator.getLeftButton().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    }

    public double getLeftX() {
        return driverLeft.getX();
    }

    public double getLeftY() {
        return driverLeft.getY();
    }

    public double getRightX() {
        return driverRight.getX();
    }
}
