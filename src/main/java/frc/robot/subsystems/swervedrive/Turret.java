package frc.robot.subsystems.swervedrive;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;

public class Turret extends SubsystemBase
{
    //Motor
    SparkMax turret = new SparkMax(11, MotorType.kBrushless);

    //Encoder
    private final RelativeEncoder turretEncoder = turret.getEncoder();

    //Limits
    private static final double MIN_TURRET_POSITION = -19;
    private static final double MAX_TURRET_POSITION = 0.0;

    //PID for aiming
    private final PIDController aimPID = new PIDController(0.03, 0.0, 0.0);

     public Turret()
    {
        aimPID.setTolerance(1.0);
    }

    public void setTurretPower(double power)
    {
        double position = getTurretAngle();

        //STOP if trying to go past left limit
        if (position <= MIN_TURRET_POSITION && power < 0)
        {
            turret.set(0.0);
            return;
        }

        //STOP if trying to go past right limit
        if (position >= MAX_TURRET_POSITION && power > 0)
        {
            turret.set(0.0);
            return;
        }
        
        turret.set(MathUtil.clamp(power, -0.35, 0.35));
    }

    public void stopTurret()
    {
        turret.set(0.0);
    }

        public void testTurnLeft()
    {
        setTurretPower(-0.15);
    }

    public void testTurnRight()
    {
        setTurretPower(0.15);
    }

    public void aimAtTarget(double yawErrorDegrees)
    {
        if(Math.abs(yawErrorDegrees) < 1.5)
        {
            stopTurret();
            return;
        }
        
        double output = aimPID.calculate(yawErrorDegrees, 0.0);

        if (Math.abs(output) < 0.08)
        {
            output = Math.copySign(0.08, output);
        }

        setTurretPower(output);
    }

    public boolean aimedAtTarget()
    {
        return aimPID.atSetpoint();
    }

    public double getTurretAngle()
    {
        return turretEncoder.getPosition();
    }

    private static double height_error(double hv, double xv, double yv,  double target_distance, double target_height, double x_target_dir, double y_target_dir, double slope) {
        double x_v_rel = hv * x_target_dir - xv;
        double y_v_rel = hv * y_target_dir - yv;
        return slope * Math.sqrt(x_v_rel * x_v_rel + y_v_rel * y_v_rel) * target_distance / hv - 4.9*target_distance*target_distance/(hv*hv) - target_height;
    }

    public Command aimWithVision(frc.robot.subsystems.swervedrive.VisionSubsystem vision, frc.robot.subsystems.swervedrive.Shooter shooter, SwerveDrive swerveDrive)
    {
        return Commands.run(() ->
        {
          
            if (vision.hasTurretHubTarget())
            {
                double yaw = vision.getTurretHubYaw();
                double pitch = vision.getTurretHubPitch();
                double distance = vision.getTurretHubDistanceMeters();

                aimAtTarget(-yaw);

                SmartDashboard.putString("Tracking", "Hub Tag Found, Yaw: " + yaw + ", Pitch: " + pitch + ", Distance: " + distance);

                //final double latency = .5; //seconds from signal to shoot to exit of ball. Probably this whole section of code should be run once to move motors and a second time after to see if correct, or if further adjustments are needed
                final double slope = 2.3298; //tan(shooting angle)

                //robot pos, assumes rotational and angular velocity remains constant
                // double r = swerveDrive.getPose().getRotation().getRadians();
                // double rv = swerveDrive.getRobotVelocity().omegaRadiansPerSecond;
                // double xv = swerveDrive.getRobotVelocity().vxMetersPerSecond;
                // double yv = swerveDrive.getRobotVelocity().vyMetersPerSecond;
                // double xp = swerveDrive.getPose().getX()+0.19685*Math.cos(r+latency*rv)+latency*xv; //TODO: check if this rotates correctly
                // double yp = swerveDrive.getPose().getY()+0.19685*Math.sin(r+latency*rv)+latency*yv;
                // xv += -0.19685*rv*Math.sin(r+latency*rv); //add on velocity from angular velocity
                // yv += 0.19685*rv*Math.cos(r+latency*rv);
                double xv = 0;
                double yv = 0;

                // SmartDashboard.putString("Pose", "xp: " + xp + ", yp: " + yp + ", xv:" + xv + ", yv: " + yv);

                // // position on field, 4.62534 meters X, 4.03479 meters Y
                // Optional<Alliance> alliance = DriverStation.getAlliance();
                // final double target_x;
                // if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                // target_x = 11.91466;
                // } else {
                // target_x = 4.62534;
                // }
                // final double target_y = 4.03479;


                //shooting velocities
                final double rpmFactor = 400;
                final double target_height = 1.8288;
                // double target_distance = Math.sqrt((target_x-xp)*(target_x-xp)+(target_y-yp)*(target_y-yp));
                double target_distance = distance;
                //double x_target_dir = (target_x - xp)/target_distance;
                //double y_target_dir = (target_y - yp)/target_distance;
                double x_target_dir = 1;
                double y_target_dir = 0;
                double hv = .1; //refined through newton's method
                for(int c=0; c<16; c++) {
                    double value = height_error(hv, xv, yv, target_distance, target_height, x_target_dir, y_target_dir, slope);
                    double increment = height_error(hv+.01, xv, yv, target_distance, target_height, x_target_dir, y_target_dir, slope);
                    hv += -.01*value / (increment - value);
                }
                double x_v_rel = hv * x_target_dir - xv;
                double y_v_rel = hv * y_target_dir - yv;
                double h_v_rel = Math.sqrt(x_v_rel*x_v_rel + y_v_rel*y_v_rel);
                double velocity = Math.sqrt(h_v_rel*h_v_rel + slope*slope*h_v_rel*h_v_rel);
                double shooting_dir = Math.atan2(y_v_rel, h_v_rel);

                //verification
                final double edge_height = target_height + .3;
                double edge_distance = target_distance-.6;
                double edge_time = edge_distance/hv;
                double height_at_edge = slope*h_v_rel*edge_time-4.9*edge_time*edge_time;
                if(height_at_edge < edge_height) {
                    SmartDashboard.putString("Targeting","TOO CLOSE! " + (height_at_edge - edge_height) + " M under edge");
                } else if(rpmFactor*velocity > 5800) {
                    SmartDashboard.putString("Targeting","TOO FAR! " + rpmFactor*velocity + " RPM wanted, probably not possible!");
                } else {
                    SmartDashboard.putString("Targeting","shooting_velocity: " + velocity + ", shooting_dir: " + shooting_dir + ", " + (height_at_edge - edge_height) + " M over edge");
                }
                shooter.setRPM(rpmFactor*velocity);
            }
            else
            {
                stopTurret();
                shooter.stopShooter();
                SmartDashboard.putString("Tracking", "HUB TAG NOT FOUND");
                SmartDashboard.putString("Targeting","needs tracking");
            }
        }, this).finallyDo(() -> stopTurret());
    }

    @Override
    public void periodic()
    {
        //System.out.println("Turret Position: " + turretEncoder.getPosition());
    } 

}
