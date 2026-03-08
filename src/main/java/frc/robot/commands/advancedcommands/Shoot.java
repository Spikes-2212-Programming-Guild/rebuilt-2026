package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.difficultcommands.ShootWithPID;
import frc.robot.commands.simplecommands.SimpleSpin;
import frc.robot.commands.simplecommands.SimpleTransport;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SpinningMagazine;
import frc.robot.subsystems.Transport;
import frc.robot.utils.ShooterAlgo;
import frc.robot.utils.VisionService;

public class Shoot extends WrapperCommand {

    private final Shooter shooter;
    private final Transport transport;
    private final SpinningMagazine spinningMagazine;

    public Shoot(Shooter shooter, Transport transport, SpinningMagazine spinningMagazine,
                 VisionService visionService) {
        super(
                new SequentialCommandGroup(
                        new ShootWithPID(shooter,
                                () -> ShooterAlgo.calculateRPM(visionService.getTargetRelativePose().getX())) {

                            @Override
                            public void end(boolean i) {}
                        },
                        new ParallelCommandGroup(
                                new SimpleSpin(spinningMagazine),
                                new SimpleTransport(transport),
                                new ShootWithPID(shooter,
                                        () -> ShooterAlgo.calculateRPM(visionService.getTargetRelativePose().getX()))
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
