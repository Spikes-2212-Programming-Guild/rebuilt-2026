package frc.robot.commands.advancedcommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.storage.Spin;
import frc.robot.commands.storage.Transport;
import frc.robot.commands.swerve.RotateAccordingAprilTags;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class Shoot extends WrapperCommand {

    private static final double M = 0.4;
    private static final double B = 2.63;
    private static final double DISTANCE_FROM_CAMERA_TO_SHOOTER = -1.0;

    private final Shooter shooter;
    private final Kicker transport;
    private final SpinningMagazine spinningMagazine;

    public Shoot(Shooter shooter, Kicker transport, SpinningMagazine spinningMagazine,
                 Drivetrain drivetrain, VisionService visionService) {

        super(
                new SequentialCommandGroup(
                        new RotateAccordingAprilTags(drivetrain, ()-> 0.0, visionService),
                        new ShootWithPID(shooter,
                                () -> M * visionService.getTargetRelativePose().getTranslation().getDistance(
                                        new Translation2d(0, DISTANCE_FROM_CAMERA_TO_SHOOTER)) + B) {

                            @Override
                            public void end(boolean i) {}
                        },
                        new ParallelCommandGroup(
                                new Spin(spinningMagazine),
                                new Transport(transport),
                                new ShootWithPID(shooter,
                                        () -> M * visionService.getTargetRelativePose().getTranslation().getDistance(
                                                new Translation2d(0, DISTANCE_FROM_CAMERA_TO_SHOOTER)) + B)
                        )
                )
        );

        this.shooter = shooter;
        this.transport = transport;
        this.spinningMagazine = spinningMagazine;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        transport.stop();
        spinningMagazine.stop();
    }
}
