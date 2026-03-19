package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.advancedcommands.*;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.swerve.RotateAccordingToGyro;
import frc.robot.subsystems.forbar.Collection;
import frc.robot.subsystems.forbar.CollectionMovement;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class OI /*GEVALD*/ {

    private final PlaystationControllerWrapper driverPlaystation = new PlaystationControllerWrapper(0);
    private final PlaystationControllerWrapper navigatorPlaystation = new PlaystationControllerWrapper(1);

    private final Collection collection = Collection.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final CollectionMovement collectionMovement = CollectionMovement.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final SpinningMagazine spinningMagazine = SpinningMagazine.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final VisionService visionService = VisionService.getInstance();

    public OI() {
        navigatorPlaystation.getL2Button().whileTrue(new Collect(collection, collectionMovement))
                .onFalse(new MoveCollectionUp(collection, collectionMovement));
        navigatorPlaystation.getR2Button().whileTrue(new Pass(shooter, () -> -1.0, spinningMagazine, kicker));
        navigatorPlaystation.getR1Button().whileTrue(new ShootToHub(shooter, kicker, spinningMagazine,
                visionService));
        navigatorPlaystation.getCircleButton().whileTrue(new MoveGenericSubsystem(collection, -0.3)).
                onFalse(new MoveGenericSubsystem(collection, () -> 0.0));
        navigatorPlaystation.getSquareButton().whileTrue(new Intake(collection)).onFalse(new MoveGenericSubsystem(
                collection, () -> 0.0));
        navigatorPlaystation.getLeftStickButton().onTrue(
                new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        driverPlaystation.getTriangleButton().onTrue(new InstantCommand(drivetrain::resetFieldRelativity));
        driverPlaystation.getR2Button().whileTrue(new TuneToAprilTag(drivetrain, visionService,
                shooter, kicker, spinningMagazine, collection, 1).
                andThen(new ShootToHub(shooter, kicker, spinningMagazine, visionService)));
        driverPlaystation.getL2Button().whileTrue(new TuneToAprilTag(drivetrain, visionService,
                shooter, kicker, spinningMagazine, collection, -1));
        driverPlaystation.getR1Button().whileTrue(new RotateAccordingToGyro(drivetrain, () -> -90.0,
                true));
        driverPlaystation.getL1Button().whileTrue(new RotateAccordingToGyro(drivetrain, () -> 270.0,
                true));

    }

    public double getLeftX() {
        return driverPlaystation.getLeftX();
    }

    public double getLeftY() {
        return driverPlaystation.getLeftY();
    }

    public double getRightX() {
        return driverPlaystation.getRightX();
    }

    public double getRightY() {
        return driverPlaystation.getRightY();
    }
}
