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
import frc.robot.subsystems.intake.Collection;
import frc.robot.subsystems.intake.CollectionMovement;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class OI /*GEVALD*/ {

    private final Joystick driverRight = new Joystick(0);
    private final Joystick driverLeft = new Joystick(1);
    private final PlaystationControllerWrapper navigator = new PlaystationControllerWrapper(2);

    private final Collection collection = Collection.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final CollectionMovement collectionMovement = CollectionMovement.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final SpinningMagazine spinningMagazine = SpinningMagazine.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final VisionService visionService = VisionService.getInstance();

    public OI() {
        configureDriver();
        configureNavigator();
    }

    private void configureDriver() {
        new JoystickButton(driverRight, 1).onTrue(new InstantCommand(drivetrain::resetFieldRelativity));
        new JoystickButton(driverRight, 3).whileTrue(new RotateAccordingToGyro(drivetrain, () -> 270.0,
                true));
        new JoystickButton(driverRight, 4).whileTrue(new RotateAccordingToGyro(drivetrain, () -> 90.0,
                true));

        new JoystickButton(driverLeft, 3).whileTrue(new TuneToAprilTag(drivetrain, visionService,
                shooter, kicker, spinningMagazine, collection, 1).
                andThen(new ShootToHub(shooter, kicker, spinningMagazine, visionService)));
        new JoystickButton(driverLeft, 4).whileTrue(new TuneToAprilTag(drivetrain, visionService,
                shooter, kicker, spinningMagazine, collection, -1).
                andThen(new ShootToHub(shooter, kicker, spinningMagazine, visionService)));
        new JoystickButton(driverLeft, 2).whileTrue(new ModuleRotateWithPID(drivetrain,
                45.0, 135.0, 135.0, 45.0));
    }

    private void configureNavigator() {
        navigator.getL1Button().whileTrue(new Shoot());
        navigator.getL2Button().whileTrue(new ShootToHub(shooter, kicker, spinningMagazine, visionService));
        navigator.getR2Button().whileTrue(new SpinCollection(0.55));
        navigator.getTriangleButton().onTrue(new UpWithJumpies(collectionMovement));
        navigator.getUpButton().whileTrue(new MoveCollection(-0.4));
        navigator.getDownButton().whileTrue(new MoveCollection(0.4));
        navigator.getCrossButton().onTrue(new MoveDown());
        navigator.getSquareButton().whileTrue(new SpinCollection(-0.75));
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

    public double getRightY() {
        return driverRight.getY();
    }
}
