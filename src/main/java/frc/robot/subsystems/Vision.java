package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import static java.lang.Math.*;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Vision extends SubsystemBase {

    public static boolean canSee;
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTable table2 = NetworkTableInstance.getDefault().getTable("limelight2");
    PhotonCamera rightCam = new PhotonCamera("RightCam");
    PhotonCamera leftCam = new PhotonCamera("LeftCam");

    //math variables
    private double d1;
    private double d2;


    @Override
    public void periodic() {
        var LCamoutput = leftCam.getLatestResult();
        var RCamoutput = rightCam.getLatestResult();

        boolean RCamTarget = RCamoutput.hasTargets();
        boolean LCamTarget = LCamoutput.hasTargets();

        //if camera has a target, get data. Code breaks if you don't check for target
        if(LCamTarget){
            var LCamTargetData = LCamoutput.getBestTarget();
            double LCamYaw = LCamTargetData.getYaw();
            double LCamPitch = LCamTargetData.getPitch();
            double LCamSkew = LCamTargetData.getSkew();
            double LCamArea = LCamTargetData.getArea();
            double LCamError = LCamTargetData.getPoseAmbiguity();

            SmartDashboard.putNumber("LeftCamError", LCamError);
            SmartDashboard.putNumber("LeftCamArea", LCamArea);
            SmartDashboard.putNumber("LeftCamSkew", LCamSkew);
            SmartDashboard.putNumber("LeftCamYaw", LCamYaw);
            SmartDashboard.putNumber("LeftCamPitch", LCamPitch);
        }

        if(RCamTarget){
            var RCamTargetData = RCamoutput.getBestTarget();
            double RCamYaw = RCamTargetData.getYaw();
            double RCamPitch = RCamTargetData.getPitch();
            double RCamSkew = RCamTargetData.getSkew();
            double RCamArea = RCamTargetData.getArea();
            double RCamError = RCamTargetData.getPoseAmbiguity();

            SmartDashboard.putNumber("RightCamYaw", RCamYaw);
            SmartDashboard.putNumber("RightCamPitch", RCamPitch);
            SmartDashboard.putNumber("RightCamSkew", RCamSkew);
            SmartDashboard.putNumber("RightCamArea", RCamArea);
            SmartDashboard.putNumber("RightCamError", RCamError);
        }


        SmartDashboard.putBoolean("LeftCamTarget?", LCamTarget);
        SmartDashboard.putBoolean("RightCamTarget?", RCamTarget);
        

        //read values periodically
        double x1 = LimelightHelpers.getTX(Constants.LLName);
        double y1 = LimelightHelpers.getTY(Constants.LLName);
        double area1 = LimelightHelpers.getTA(Constants.LLName);

        double x2 = LimelightHelpers.getTX(Constants.LL2Name);
        double y2 = LimelightHelpers.getTY(Constants.LL2Name);
        double area2 = LimelightHelpers.getTA(Constants.LL2Name);

        if(area1 == 1 || area2 == 1){
            canSee = true;
        }
        else{
            canSee = false;
        }
//---------------------------------------------------------------------------------
//------------------- MATH! (Transcribed from Mr. Tice's notes) -------------------
//---------------------------------------------------------------------------------

        /* -Requires tx and ty for 2 ATs
         * -from previous notes, calculate distance to both AT1 and AT2 | DO NOT ASSUME THAT TX = 0
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
         * α = (π / 2) - ΘAT1
         * α = -(π / 2) + Θ▲ + ΘAT2
         * 
         * x = d1 sin(α)
         * y = d1 cos(α)
         * 
         * ((SinΘ AT2) / (d1)) = ((SinΘ▲) / (dAT))
         * (Law of Sines) = Θa2 = arcsin((d1 /d2) * sinΘ▲)
         * 
         * α = -(π / 2) + ▲ - arcsin((d1 / dAT) * sin(Θ▲))
         */

        
        //Xnormalized = tan(x1) / sqrt(1 + (pow(tan(y1), 2)) + (pow(tan(x1), 2)));

        //Ynormalized = tan(y1) / sqrt(1 + (pow(tan(y1), 2)) + (pow(tan(x1), 2)));

        //Scale = (targetheight - ShooterConstants.CamHeight) / Ynormalized;

        // Distance = sqrt(pow(Xnormalized, 2) + pow(Znormalized, 2)) * Scale;
        
        //Distance calculations for each LL
        d1 = getDistancev2(Constants.LLName);
        d2 = getDistancev2(Constants.LL2Name);


//---------------------------------------------------------------------------------

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x1);
        SmartDashboard.putNumber("LimelightY", y1);
        SmartDashboard.putNumber("Limelight2X", x2);
        SmartDashboard.putNumber("Limelight2Y", y2);
        SmartDashboard.putNumber("LimelightArea", area1);
        SmartDashboard.putNumber("LLd1", d1);
        SmartDashboard.putNumber("LLd2", d2);
        SmartDashboard.putBoolean("cansee?", canSee);
    }
    


    @Override
    public void simulationPeriodic() {
        
    }

    public static int roundDistance(String llName){
        //takes distance (currently subed for .getTa), and rounds it to the nearest whole number (casted to an int because round returns long)
        int distance = (int) round(getDistance(llName));
        return distance;
    }

    public static double getDistance(String llName){
    //https://docs.wpilib.org/en/latest/docs/software/vision-processing/introduction/identifying-and-processing-the-targets.html#distance
    //uses this equation ^
    //distance = (targetheight - cameraheight) / tan(cameraangle + Ty)
        var y = LimelightHelpers.getTY(llName);
        double distance = (ShooterConstants.ApTagHeight - ShooterConstants.CamHeight) / Math.tan((ShooterConstants.CamAngle + (y)) * (Math.PI/180));
        return distance;
   }

   public static double getDistancev2(String llName){
      var y = LimelightHelpers.getTY(llName);
      var x = LimelightHelpers.getTX(llName);
      var deltaH = ShooterConstants.ApTagHeight - ShooterConstants.CamHeight;

      double distance = (deltaH / (tan(y) * cos(x)));
      return distance;
   }
}