package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.storage.Spin;
import frc.robot.commands.storage.Transport;
import frc.robot.commands.swerve.SwerveRotateWithPID;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.ShooterAlgorithm;
import frc.robot.utils.VisionService;

public class Shoot extends WrapperCommand {

    private final Shooter shooter;
    private final Kicker transport;
    private final SpinningMagazine spinningMagazine;

    public Shoot(Shooter shooter, Kicker transport, SpinningMagazine spinningMagazine,
                 Drivetrain drivetrain, VisionService visionService) {
        super(
                new SequentialCommandGroup(
                        new ShootWithPID(shooter,
                                () -> ShooterAlgorithm.calculateRPM(visionService.getTargetRelativePose().getX())) {

                            @Override
                            public void end(boolean i) {}
                        },
                        new ParallelCommandGroup(
                                new Spin(spinningMagazine),
                                new Transport(transport),
                                new ShootWithPID(shooter,
                                        () -> ShooterAlgorithm.calculateRPM(visionService.getTargetRelativePose().getX()))
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
