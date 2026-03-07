package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.ShootWithPID;
import frc.robot.commands.simplecommands.SimpleSpin;
import frc.robot.commands.simplecommands.SimpleTransport;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SpinningMagazine;
import frc.robot.subsystems.Transport;
import frc.robot.utils.ShooterAlgo;
import frc.robot.utils.VisionService;

import java.util.function.Supplier;

public class Shoot extends ParallelCommandGroup {

    public Shoot(Shooter shooter, Transport transport, SpinningMagazine spinningMagazine,
                 VisionService visionService) {

        Supplier<Double> shooterSpeed = ()-> ShooterAlgo.calculateRPM(visionService.getTargetRelativePose().getX());

        addCommands(
                new SequentialCommandGroup(
                        new ShootWithPID(shooter, shooterSpeed)
                ),
                new ParallelCommandGroup(
                        new SimpleSpin(spinningMagazine),
                        new SimpleTransport(transport),
                        new ShootWithPID(shooter, shooterSpeed)
                ).finallyDo((interrupted) -> {
                    shooter.stop();
                    transport.stop();
                    spinningMagazine.stop();
                })

        );
    }
}
