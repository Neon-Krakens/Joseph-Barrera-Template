package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import swervelib.SwerveDrive;
import java.util.Optional;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  private static Robot instance;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Timer disabledTimer;
  XboxController controller = new XboxController(0);



  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  private static double height_error(double hv, double xv, double yv,  double target_distance, double target_height, double x_target_dir, double y_target_dir, double slope) {
      double x_v_rel = hv * x_target_dir - xv;
      double y_v_rel = hv * y_target_dir - yv;
      return slope * Math.sqrt(x_v_rel * x_v_rel + y_v_rel * y_v_rel) * target_distance / hv - 4.9*target_distance*target_distance/(hv*hv) - target_height;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();
    //m_robotContainer.shooter.setRPM(30*SmartDashboard.getNumber("RPM Factor", 0.));
    final double latency = .5; //seconds from signal to shoot to exit of ball. Probably this whole section of code should be run once to move motors and a second time after to see if correct, or if further adjustments are needed
    final double slope = 2.3298; //tan(shooting angle)
    SwerveDrive swerveDrive = m_robotContainer.drivebase.swerveDrive;
    //robot pos, assumes rotational and angular velocity remains constant
    double r = swerveDrive.getPose().getRotation().getRadians();
    double rv = swerveDrive.getRobotVelocity().omegaRadiansPerSecond;
    double xv = swerveDrive.getRobotVelocity().vxMetersPerSecond;
    double yv = swerveDrive.getRobotVelocity().vyMetersPerSecond;
    double xp = swerveDrive.getPose().getX()+0.19685*Math.cos(r+latency*rv)+latency*xv; //TODO: check if this rotates correctly
    double yp = swerveDrive.getPose().getY()+0.19685*Math.sin(r+latency*rv)+latency*yv;
    xv += -0.19685*rv*Math.sin(r+latency*rv); //add on velocity from angular velocity
    yv += 0.19685*rv*Math.cos(r+latency*rv);

    SmartDashboard.putString("Pose", "xp: " + xp + ", yp: " + yp + ", xv:" + xv + ", yv: " + yv);

    // position on field, 4.62534 meters X, 4.03479 meters Y
    Optional<Alliance> alliance = DriverStation.getAlliance();
    final double target_x;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
    target_x = 11.91466;
    } else {
    target_x = 4.62534;
    }
    final double target_y = 4.03479;


    //shooting velocities
    final double rpmFactor = SmartDashboard.getNumber("RPM Factor", 1200);
    final double target_height = 1.8288;
    double target_distance = Math.sqrt((target_x-xp)*(target_x-xp)+(target_y-yp)*(target_y-yp));
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
    m_robotContainer.shooter.setRPM(rpmFactor*velocity);
    m_robotContainer.turret.aimAtTarget(-((shooting_dir - r + 1.5708) % 3.14159265359 - 1.5708));
    //m_robotContainer.turret.aimAtTarget(0.2);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    disabledTimer.reset();
    disabledTimer.start();
  }
  
  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //Print the selected autonomous command upon autonomous init
    System.out.println("Auto selected: " + m_autonomousCommand);

    // schedule the autonomous command selected in the autoChooser
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.getAutonomousCommand();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }

    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {

  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {

  }
}
