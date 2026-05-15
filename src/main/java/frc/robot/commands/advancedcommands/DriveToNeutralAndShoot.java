package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.DriveDistanceWithPID;
import frc.robot.commands.swerve.DriveDistanceWithPID.DriveDirection;
import frc.robot.commands.swerve.SwerveRotateWithPID;
import frc.robot.subsystems.forbar.Collection;
import frc.robot.subsystems.forbar.CollectionMovement;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;

/*
    init_pos: 3.75m, 2.5m  # between hub and trench
    init_angle: shooter facing other alliance
 */
public class DriveToNeutralAndShoot extends SequentialCommandGroup {

    public DriveToNeutralAndShoot(Drivetrain drivetrain, Shooter shooter,
                                  SpinningMagazine spinningMagazine, Kicker kicker,
                                  Collection collection, CollectionMovement collectionMovement) {
        addCommands(
                new DriveDistanceWithPID(drivetrain, -1, DriveDirection.X, false), // drive to shooting position
                new SwerveRotateWithPID(drivetrain, () -> null, false), // rotate to shoot
                new AccelerateAndShoot(shooter, () -> null, spinningMagazine, kicker)
                        .withTimeout(-1), // shoot until all balls are shot

                new SwerveRotateWithPID(drivetrain, () -> 90.0, false), // rotate to drive to trench
                new DriveDistanceWithPID(drivetrain, -1.75, DriveDirection.Y, false), // drive to align with trench
                new DriveDistanceWithPID(drivetrain, 5, DriveDirection.X, false), // drive to neutral zone

                new ParallelDeadlineGroup( // keep collecting until drove the entire way
                        new Collect(collection, collectionMovement),
                        new DriveDistanceWithPID(drivetrain, 1.5, DriveDirection.Y, false)
                ),

                new DriveDistanceWithPID(drivetrain, -1.5, DriveDirection.Y, false), // drive to align with trench
                new DriveDistanceWithPID(drivetrain, -5.5, DriveDirection.X, false), // drive to alliance zone

                new DriveDistanceWithPID(drivetrain, 2, DriveDirection.Y, false), // drive to shooting position
                new SwerveRotateWithPID(drivetrain, () -> null, false), // rotate to shoot
                new AccelerateAndShoot(shooter, () -> null, spinningMagazine, kicker)
                        .withTimeout(-1) // shoot until all balls are shot

        );
    }
}
