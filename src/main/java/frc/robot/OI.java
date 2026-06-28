package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.advancedcommands.*;
import frc.robot.commands.intake.SpinCollection;
import frc.robot.commands.shoot.JustShoot;
import frc.robot.commands.storage.SpinMagazine;
import frc.robot.commands.storage.Transport;
import frc.robot.commands.swerve.RotateAccordingToGyro;
import frc.robot.subsystems.intake.Collection;
import frc.robot.subsystems.intake.CollectionMovement;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

import java.security.cert.PolicyQualifierInfo;

public class OI /*GEVALD*/ {

    private final Joystick driverRight = new Joystick(0);
    private final Joystick driverLeft = new Joystick(1);
    private final PlaystationControllerWrapper navigator = new PlaystationControllerWrapper(2);
    private final PlaystationControllerWrapper dr = new PlaystationControllerWrapper(3);

    private final Collection collection = Collection.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final CollectionMovement collectionMovement = CollectionMovement.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final SpinningMagazine spinningMagazine = SpinningMagazine.getInstance();
    private final Kicker kicker = Kicker.getInstance();
//    private final VisionService visionService = VisionService.getInstance();

    public OI() {
        configureDriver();
//        configureNavigator();
    }

    private void configureDriver() {
        dr.getR2Button().onTrue(new InstantCommand((drivetrain::resetFieldRelativity)));
        new JoystickButton(driverRight, 1).onTrue(new InstantCommand(drivetrain::resetFieldRelativity));
//        driver.getR2Button().whileTrue(new TuneToAprilTag(drivetrain, visionService,
//                shooter, kicker, spinningMagazine, collection, 1).
//                andThen(new ShootToHub(shooter, kicker, spinningMagazine, visionService)));
//        driver.getL2Button().whileTrue(new TuneToAprilTag(drivetrain, visionService,
//                shooter, kicker, spinningMagazine, collection, -1));
//        driver.getR1Button().whileTrue(new JustShoot(shooter, () -> 0.6).withTimeout(1)
//                .andThen(new JustShoot(shooter, () -> 0.6).alongWith(new Transport(kicker),
//                        new SpinMagazine(spinningMagazine))));
//        driver.getL1Button().whileTrue(new RotateAccordingToGyro(drivetrain, () -> 270.0,
//                true));
    }

    private void configureNavigator() {
//        navigator.getL2Button().whileTrue(new Collect(collection, collectionMovement))
//                .onFalse(new MoveCollectionUp(collection, collectionMovement));
//        navigator.getR2Button().whileTrue(new Pass(shooter, () -> -1.0, spinningMagazine, kicker));
//        navigator.getR1Button().whileTrue(new ShootToHub(shooter, kicker, spinningMagazine,
//                visionService));
//        navigator.getCircleButton().whileTrue(new MoveGenericSubsystem(collection, -0.05)).
//                onFalse(new MoveGenericSubsystem(collection, () -> 0.0));
//        navigator.getSquareButton().whileTrue(new SpinCollection(collection)).onFalse(new MoveGenericSubsystem(
//                collection, () -> 0.0));
//        navigator.getLeftStickButton().onTrue(
//                new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    }
//
//    public double getLeftX() {
//        return driverLeft.getX();
//    }
//
//    public double getLeftY() {
//        return driverLeft.getY();
//    }
//
//    public double getRightX() {
//        return driverRight.getX();
//    }
//
//    public double getRightY() {
//        return driverRight.getY();
//    }


    public double getLeftX() {
        return dr.getLeftX();
    }

    public double getLeftY() {
        return dr.getLeftY();
    }

    public double getRightX() {
        return dr.getRightX();
    }

    public double getRightY() {
        return dr.getRightY();
    }

}
