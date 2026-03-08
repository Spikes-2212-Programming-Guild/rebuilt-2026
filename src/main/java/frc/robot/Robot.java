// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.advancedcommands.Collect;
import frc.robot.commands.advancedcommands.UpSlowly;
import frc.robot.commands.simplecommands.MoveCollection;
import frc.robot.commands.simplecommands.SimpleIntake;
import frc.robot.commands.simplecommands.SimpleShoot;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {

    private static final RootNamespace namespace = new RootNamespace("robot");
    private final Shooter shooter = Shooter.getInstance();
    private final CollectionMovement collectionMovement = CollectionMovement.getInstance();
    private final Collection collection = Collection.getInstance();

    @Override
    public void robotInit() {
        namespace.putCommand("move collection up", new UpSlowly(collectionMovement));
        namespace.putCommand("move collection down", new MoveCollection(collectionMovement, ()-> -0.05).withTimeout(1));
        namespace.putCommand("move up slow", new MoveCollection(collectionMovement, ()-> 0.1));
        namespace.putCommand("shoot", new SimpleShoot(shooter, ()-> -0.3));
        namespace.putCommand("collect", new SimpleIntake(collection));
        namespace.putCommand("re lease", new MoveGenericSubsystem(collection, -0.2));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        namespace.update();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        OI oi = new OI();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
