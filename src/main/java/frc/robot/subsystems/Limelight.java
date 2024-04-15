package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import static java.lang.Math.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight extends SubsystemBase {

    public static boolean canSee;
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");



    @Override
    public void periodic() {
        //read values periodically
        double x = LimelightHelpers.getTX(Constants.LLName);
        double y = LimelightHelpers.getTY(Constants.LLName);
        double area = LimelightHelpers.getTA(Constants.LLName);

        if(area == 0){
            canSee = false;
        }
        else{
            canSee = true;
        }
//---------------------------------------------------------------------------------
//------------------- MATH! (Transcribed from Mr. Tice's notes) -------------------
//---------------------------------------------------------------------------------
        /* -Requires tx and ty for 2 ATs
         * -from previous notes, calculate distance to both AT1 and AT2 DO NOT ASSUME THAT TX = 0
         * 
         * 
         * Xnormalized = (tan(tx)) / (√1 + tan²(ty) + tan²(tx))
         * 
         * Ynormalized = (tan(ty)) / (√1 + tan²(ty) + tan²(tx))
         * 
         * Scale = ((Target Height) - (Camera Height)) / Ynormalized
         *  = (▲H) / (Ynormalized)
         * 
         * Distance = ((√x²normal + z²norm) * scale)
         *  = (▲H) / (tan(ty) * cos(tx))
         * 
         * SEE NOTES FOR DIAGRAM
         * 
         * Θ = π - Θ▲ - Θa2 (law of sum of angles)
         * 
         * a = (π / 2) - ΘAT1
         * a = -(π / 2) + Θ▲ + ΘAT2
         * 
         * ((SinΘ AT2) / (d1)) = ((SinΘ▲) / (dAT))
         * (Law of Sines) = Θa2 = arcsin((d1 /d2) * sinΘ▲)
         * 
         * a = -(π / 2) + ▲ - arcsin((d1 / dAT) * sin(Θ▲))
         */



//---------------------------------------------------------------------------------

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("GetDistance", getDistance());
        SmartDashboard.putBoolean("cansee?", canSee);
    }
    


    @Override
    public void simulationPeriodic() {
        
    }

    public static int roundDistance(){
        //takes distance (currently subed for .getTa), and rounds it to the nearest whole number (casted to an int because round returns long)
        int distance = (int) round(getDistance());
        return distance;
    }

    public static double getDistance(){
    //https://docs.wpilib.org/en/latest/docs/software/vision-processing/introduction/identifying-and-processing-the-targets.html#distance
    //uses this equation ^
    //distance = (targetheight - cameraheight) / tan(cameraangle + Ty)
        var y = LimelightHelpers.getTY(Constants.LLName);
        double distance = (ShooterConstants.ApTagHeight - ShooterConstants.CamHeight) / Math.tan((ShooterConstants.CamAngle + (y)) * (Math.PI/180));
        return distance;
   }
}