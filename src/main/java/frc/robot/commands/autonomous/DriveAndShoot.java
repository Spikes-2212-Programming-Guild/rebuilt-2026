package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.advancedcommands.RotateToTag;
import frc.robot.commands.advancedcommands.ShootToHub;
import frc.robot.commands.advancedcommands.ShootWPIDAuto;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;

public class DriveAndShoot extends SequentialCommandGroup {

    private static final double SPEED_TO_DRIVE_X_AXIS = 2.5;
    private static final double TIME_TO_DRIVE = 0.7;

    public DriveAndShoot(Drivetrain drivetrain) {
        addCommands(
                new InstantCommand(() -> drivetrain.setGyro(90)),
                new Drive(drivetrain, () -> SPEED_TO_DRIVE_X_AXIS, () -> 0.0,
                        () -> 0.0, true, true).withTimeout(TIME_TO_DRIVE),
                new RotateToTag(1).withTimeout(0.5),
                new ShootWPIDAuto()
        );
    }
}
