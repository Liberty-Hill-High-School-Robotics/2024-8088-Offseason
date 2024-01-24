package frc.robot.subsystems;


//imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class Pivot extends SubsystemBase {

    //motors & variables here
    private CANSparkMax pivotSparkMax;
    private CANSparkMax pivotSparkMax2;
    private CANSparkMax transferRollerSparkMax;



    public Pivot(){
        //config motor settings here
        pivotSparkMax = new CANSparkMax(CanIDs.pivotMotorID, MotorType.kBrushless);
        pivotSparkMax.restoreFactoryDefaults();
        pivotSparkMax.setInverted(false);
        pivotSparkMax.setIdleMode(IdleMode.kBrake);

        pivotSparkMax2 = new CANSparkMax(CanIDs.pivotMotor2ID, MotorType.kBrushless);
        pivotSparkMax2.restoreFactoryDefaults();
        pivotSparkMax2.setInverted(false);
        pivotSparkMax2.setIdleMode(IdleMode.kBrake);

        transferRollerSparkMax = new CANSparkMax(CanIDs.transferRollerID, MotorType.kBrushless);
        transferRollerSparkMax.restoreFactoryDefaults();
        transferRollerSparkMax.setInverted(false);
        transferRollerSparkMax.setIdleMode(IdleMode.kCoast);
    }

  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void pivotForward(){
        pivotSparkMax.set(MotorSpeeds.pivotSpeed);
        pivotSparkMax2.set(MotorSpeeds.pivotSpeed);
    }

    public void pivotBackwards(){
        pivotSparkMax.set(-MotorSpeeds.pivotSpeed);
        pivotSparkMax2.set(-MotorSpeeds.pivotSpeed);
    }

    public void pivotStop(){
        pivotSparkMax.set(0);
        pivotSparkMax2.set(0);
    }

    //transfer roller
    public void tRollerForward(){
        transferRollerSparkMax.set(MotorSpeeds.transferRollerSpeed);
    }

    public void tRollerBackward(){
        transferRollerSparkMax.set(-MotorSpeeds.transferRollerSpeed);
    }

    public void tRollerStop(){
        transferRollerSparkMax.set(0);
    }


}
