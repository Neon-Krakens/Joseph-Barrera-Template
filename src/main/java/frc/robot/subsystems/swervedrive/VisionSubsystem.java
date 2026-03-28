package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionSubsystem extends SubsystemBase
{
    
    private final PhotonCamera turretCam = new PhotonCamera("turretCam");
    private final PhotonCamera bodyCam = new PhotonCamera("bodyCam");

    public VisionSubsystem()
    {
    }

    public PhotonPipelineResult getTurretResult()
    {
        return turretCam.getLatestResult();
    }

    public PhotonPipelineResult getBodyResult()
    {
        return bodyCam.getLatestResult();
    }

   private boolean isHubTag(int id)
    {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return (id == 25 || id == 26 || id == 27 || id == 24);
        } else {
            return (id == 9 || id == 10 || id == 8 || id == 11);
        }
    }

    public boolean hasTurretHubTarget()
    {
        var result = turretCam.getLatestResult();

        if (!result.hasTargets())
    {
        return false;
    }

    for (var target : result.getTargets())
    {
        if (isHubTag(target.getFiducialId()))
        {
            return true;
        }
    }

    return false;
    }

    public double getTurretHubYaw()
    {
    var result = turretCam.getLatestResult();

    if (!result.hasTargets())
    {
        return 0.0;
    }

    for (var target : result.getTargets())
    {
        if (isHubTag(target.getFiducialId()))
        {
            return target.getYaw();
        }
    }

    return 0.0;
    }

    public double getTurretHubPitch()
    {
        var result = turretCam.getLatestResult();
        if(!result.hasTargets())
        {
            return 0.0;
        }
        for (var target : result.getTargets())
        {
            if(isHubTag(target.getFiducialId()))
            {
                return target.getPitch();
            }
        }

        return 0.0;
    }

    public double getTurretHubDistanceMeters()
    {
        //CHANGE THIS LATER TO REAL NUMBERS
        double cameraHeightMeters = 0.5588;
        double targetHeightMeters = 1.1684;
        double cameraAngleDegrees = 28.0;

        double pitchDegrees = getTurretHubPitch();
        double totalAngleDegrees = cameraAngleDegrees + pitchDegrees;

        if(Math.abs(totalAngleDegrees) < 0.001)
        {
            return 0.0;
        }

        return (targetHeightMeters - cameraHeightMeters) / Math.tan(Math.toRadians(totalAngleDegrees));
    }

    // public boolean hasTurretTarget()
    // {
    //     return getTurretResult().hasTargets();
    // }

    // public double getTurretYaw()
    // {
    //     if (hasTurretTarget())
    //     {
    //         return getTurretResult().getBestTarget().getYaw();
    //     }
    //     return 0.0;
    // }

    // public double getTurretPitch()
    // {
    //     if (hasTurretTarget())
    //     {
    //         return getTurretResult().getBestTarget().getPitch();
    //     }
    //     return 0.0;
    // }


}
