package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.advancedcommands.RotateToTag;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.Transport;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.swerve.Drivetrain;

public class HubAuto extends SequentialCommandGroup {

    private static final double SPEED_TO_DRIVE_X_AXIS = 2.5;
    private static final double TIME_TO_DRIVE = 1;
    private static final double SHOOT_SPEED = 3.2;

    public HubAuto() {
        addCommands(
                new InstantCommand(() -> Drivetrain.getInstance().setGyro(90)),
                new Drive(() -> SPEED_TO_DRIVE_X_AXIS, () -> 0.0,
                        () -> 0.0, true, true).withTimeout(TIME_TO_DRIVE),
//                new RotateToTag(1).withTimeout(0.5),
                new ShootWithPID(SHOOT_SPEED, 0.5).withTimeout(4),
                new Transport().withTimeout(0.5),
                new ParallelCommandGroup(
                        new Transport(),
                        new SpinMagazine(0.05),
                        new ShootWithPID(SHOOT_SPEED, 15)
                )
        );
    }
}
