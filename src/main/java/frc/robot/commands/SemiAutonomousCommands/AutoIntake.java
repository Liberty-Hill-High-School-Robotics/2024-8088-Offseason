package frc.robot.commands.SemiAutonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeCommands.IntakeTogether;
import frc.robot.commands.LEDCommands.Animations.CandleLarson;
import frc.robot.commands.LEDCommands.Colors.CandleDimPurple;
import frc.robot.commands.PivotCommmands.Pivot.AngleAndFeed;
import frc.robot.commands.StorageCommands.StorageRollersFeed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.LEDs;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoIntake extends ParallelCommandGroup {


    public AutoIntake(
        Intake m_intake,
        Storage m_storage,
        Pivot m_pivot,
        LEDs m_leds
    ){
    
        addCommands(
            //run all commands in parallel until the throughbeam == true
            new IntakeTogether(m_intake),
            new AngleAndFeed(m_pivot),
            new StorageRollersFeed(m_storage),
            new CandleDimPurple(m_leds).andThen(new CandleLarson(m_leds))
            );

        }


    @Override
    public boolean runsWhenDisabled() {

        return false;
    }
}