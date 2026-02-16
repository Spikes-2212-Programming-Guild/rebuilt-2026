package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.RotateHood;
import frc.robot.commands.difficultcommands.ShootWithPID;
import frc.robot.commands.simplecommands.SimpleShoot;
import frc.robot.commands.simplecommands.SimpleSpin;
import frc.robot.commands.simplecommands.SimpleTransport;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SpinningMagazine;
import frc.robot.subsystems.Transport;

import java.util.function.Supplier;


public class Pass extends SequentialCommandGroup {

    public Pass(SpinningMagazine spinningMagazine, Hood hood, Transport transport,
                Shooter shooter, Hood.HoodPose pose, Supplier<Double> shootingSpeed){
        addCommands(
                new ParallelCommandGroup(
                        new RotateHood(hood, pose),
                        new ShootWithPID(shooter, shootingSpeed)
                ),
                new ParallelCommandGroup(
                    new SimpleSpin(spinningMagazine),
                    new SimpleTransport(transport),
                    new SimpleShoot(shooter, shootingSpeed)
                )
        );
    }
}
