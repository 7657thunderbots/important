package frc.robot;
 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {
  final double bkP = -0.0085;
     final double bkI = -0.005;
     final double bkD = -0.001;
    // final double bkI = -0.00;
    // final double bkD = -0.00;
    final double biLimit = 3;
    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;
    private double Speedvar=0.0;
    private double turnerror =0.0;
    double berror=0;
    double errorRate=0;
    public final Timer wait = new Timer();


    final double tkP = -0.0045;
    final double tkI = -0.005;
    final double tkD = -0.001;
   final double tiLimit = 3;
   double tsetpoint = 0;
   double terrorSum = 0;
   double tlastError = 0;
   double terror=0;
   double terrorRate=0;


    private static final String kDefaultSpeed = "Demo";
        private static final String kCompetitionSpeed = "Competition";
        private String speed_selected;
        private final SendableChooser<String> speed_chooser = new SendableChooser<>();


    //limelight readings
    private double tv = 0.0;
    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;
    private double distanceToTarget = -1.0;  //distance to target in inches, -1.0 means no target in sight
    private boolean autoTargeting = false;
    private boolean targetSighted = false;
    private boolean targetInRange = false;
    private boolean targetAimed = false;
    private boolean targetLocked = false;
    private boolean floppywrist = false;
    
    //These need to be set to the height of the limelight, the angle of the limelight, and the height of the target
    static final double limelightHeight = 7.5;  // measure in inches to center of lens
    static final double limelightAngle = 25.0;    // measure in degrees leaned back from vertical
    static final double targetHeight = 42;     // measure in inches
    static final double aimAdjust = -0.1;
    static final double distanceAdjust = -0.1;
    static final double min_aim_command = 0.05;
    static final double MinTargetRange = 36.0;  // no closer than 10 feet
    static final double MaxTargetRange = 40.0;  // no farther than 11 feet
    
    // the following values are used to compute a drive speed relative to the distance away from the target, the closer we get the slower we want to go
    public double MaxDriveSpeed = 0.3;  // this is the max speed we want a drive motor to run at
    static final double MinDriveSpeed = 0.1;  // this is the min speed we want a drive motor to run at
    static final double MaxDriveDistance = 144.0;  // this is the distance where we want the drive motors to be at max speed
    static final double MinDriveDistance = 1.0;  // this is the distance where we want the drive motors to be at min speed


    // the following values are used to compute a turning speed relative to the degrees we are offset from the target, the more in line we get the slower we want to turn
    static final double MaxTurnSpeed = 0.1;  // this is the max speed we want a drive motor to run at
    static final double MinTurnSpeed = 0.0;  // this is the min speed we want a drive motor to run at
    static final double MaxTurnDegrees = 27.0;  // this is the distance where we want the drive motors to be at max speed
    static final double MinTurnDegrees = 1.0;  // this is the distance where we want the drive motors to be at min speed
    
    public double TargetCenter = (MaxTargetRange + MinTargetRange)/2.0;    // the center of the target range









     double akP = 0.25;
    final double akI = 0;
    final double akD = 0.0;
  final double aiLimit = .5;
   public double dsetpoint=0;
   private double derrorSum = 0;
   private double dlastError=0;
    private double dlastTimestamp = 0;
    double dsensorPosition=0;

  private double doutputSpeed;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private boolean placed ;
  private boolean waited;
  private boolean waited2;
  private boolean try_balancing;
  private boolean taxied;
  private boolean place2= false;
  private boolean fianl_driveup;
  private boolean start_position;
  double derror;
  double derrorRate;
  double dt;
  double sensorPosition;
  double turn;
  double tsensorPosition;
  private boolean turned=false;
  private boolean armdown=false;
  private boolean turncommand=false;
  private boolean grabpiece2=false;
    int timer;
    
    
    public double speedMult;

    private final Timer m_timer = new Timer();
    public Joystick left;
    public Joystick right;
    public XboxController controller2;
    private boolean onchargestation= false;
    
    public DriveTrain drivetrain;
    
    private Hand Hand;

    private Wrist wrist;

    private Elbow elbow;

    private Shoulder shoulder;

   // private Balancing balancing;

  // private turnadjust turn;
    
   private Pneumatics pneumatics;

   private color_sensor color_sensor;

   private Auto1 auto1;

   //private Auto2_balance auto2_balance;

   private Auto3 auto3;

  //  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

   

    @Override
  public void robotInit() {
    
    speed_chooser.setDefaultOption("DemoSpeed", kDefaultSpeed);
    speed_chooser.addOption("Competition Speed", kCompetitionSpeed);
    SmartDashboard.putData("Speed choices", speed_chooser);
    place2 = false;
    placed = false;
    speedMult = .7;
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
     // This creates our drivetrain subsystem that contains all the motors and motor control code
     
     drivetrain = new DriveTrain();
      
     elbow = new Elbow();

     wrist = new Wrist();

      shoulder = new Shoulder();

      Hand = new Hand();
     
     pneumatics = new Pneumatics();

   // balancing = new Balancing();

    //  color_sensor = new color_sensor();

    //  auto1 = new Auto1();
    
    // auto2_balance = new Auto2_balance();
    
    //  auto3 = new Auto3();

    left = new Joystick(0);
		right = new Joystick(1);
		controller2 = new XboxController(2);
   drivetrain.m_gyro.reset();
   pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  

  @Override
  public void robotPeriodic() { 
   shoulder.Shoulder_Run();
    elbow.ElbowRun();  
  SmartDashboard.getNumber("elbow", elbow.Elbowencoder.getPosition());
  SmartDashboard.getNumber("Shoulder", shoulder.shouldere.getPosition());
   Hand.Hand_Run();
   wrist.Wrist_Run();
  drivetrain.run_drive();
  drivetrain.getAverageEncoderDistance();
  pneumatics.Run_Pneumatics();
  SmartDashboard.putNumber("tilt angle",drivetrain.m_gyro.getYComplementaryAngle());
  SmartDashboard.putNumber("foward distance", drivetrain.getAverageEncoderDistance());
  //SmartDashboard.putNumber("b",drivetrain.m_gyro.getXComplementaryAngle());
  SmartDashboard.putNumber("Turn angle", drivetrain.m_gyro.getAngle());
  SmartDashboard.putNumber("distance", dsensorPosition);
  SmartDashboard.putNumber("output", doutputSpeed);
  dsensorPosition=drivetrain.getAverageEncoderDistance();
  }
  


  
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    //  m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    drivetrain.m_gyro.reset();
    m_timer.reset();
		m_timer.start();
    drivetrain.setbrake(false);
    pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
     Hand.hsetpoint=-20;
    waited=false;
    waited2=false;
    onchargestation=false;
    try_balancing=false;
    dsetpoint=0;
    fianl_driveup=false;
    start_position=false;
    tsetpoint=0;
    turned=false;
    turncommand=false;
   grabpiece2=false;
    
  }


  @Override
  public void autonomousPeriodic() {
    DataLogManager.start();
    
    switch (m_autoSelected) {
        case kCustomAuto:
        dt = Timer.getFPGATimestamp() - lastTimestamp;
        if (start_position==false){
            
          elbow.Esetpoint=-39.071121;
          shoulder.Ssetpoint = 150.377;
          elbow.EkP=0.05;
          start_position=true;
        }
         // calculations
          derror = dsetpoint - dsensorPosition;
         if (Math.abs(derror) < aiLimit) {
          derrorSum += derror * dt;
         }
 
          derrorRate = (derror - dlastError) / dt;
 
          doutputSpeed = (akP * derror + akI * derrorSum + akD * derrorRate);

          if (dsetpoint==0){
            if (doutputSpeed < -.6){
              doutputSpeed=-.6;
            }
            if (doutputSpeed>-.2){
              doutputSpeed=-.2;
            }
          }
         //drivetrain.tankDrive(doutputSpeed,doutputSpeed,false);
        //  drivetrain.mywatchdog();
        //  SmartDashboard.getNumber("output", doutputSpeed);
         // output to motors
        // double Speedvar = doutputSpeed;
         // update last- variables
         lastTimestamp = Timer.getFPGATimestamp();
         dlastError = derror;
         
         if (((Math.abs(dsetpoint-dsensorPosition))<.05)){
           placed=true;
           doutputSpeed =0;
          //  derrorSum = 0;
          //  lastTimestamp = 0;
          //  dlastError = 0;
          }
          // drivetrain.tdrive(doutputSpeed, doutputSpeed,false);
        
        if (placed == true && elbow.Elbowencoder.getPosition()<-30){
           wrist.Wsetpoint=-20;
          placed=false;
          place2=true;
        }
        

        if (wrist.wriste.getPosition() < -13 && place2==true ){
          waited=true;
          place2=false;
         
        }
        if (waited==true){
          Hand.hsetpoint = 0;
          pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        
        }
        if (waited==true && Hand.hande.getPosition()> -3 ){
          wrist.Wsetpoint=0;
          waited=false;
          akP=.2;
          dsetpoint=1;
          waited2=true;
        }

        if (dsetpoint==1&&armdown==false){
          if (doutputSpeed > .25){
            doutputSpeed=.25;
          }
          if (doutputSpeed<.2){
            doutputSpeed=.2;
          }
          armdown=true;
        }

        if(elbow.Elbowencoder.getPosition()>-4&&armdown==true){
          dsetpoint=3.6;
        }
        if (dsetpoint==3.6&&turned==false){
          if (doutputSpeed > .25){
            doutputSpeed=.25;
          }
          if (doutputSpeed<.15){
            doutputSpeed=.15;
          }
        }
        if (dsetpoint==2.6&&turned==true){
          if (doutputSpeed > .25){
            doutputSpeed=.25;
          }
          if (doutputSpeed<.15){
            doutputSpeed=.15;
          }
        }

        if (waited2==true && dsensorPosition>.5 && dsensorPosition<.5){
          akP=.25;
        }
       
        if (waited2==true && dsensorPosition > .35 && fianl_driveup==false) {
        shoulder.Ssetpoint=0;
        elbow.EkP=0.015;
        elbow.Esetpoint = 0;
        shoulder.Ssetpoint=0;
        }
        if (waited2==true && dsetpoint==3.6 && ((Math.abs(dsetpoint-dsensorPosition))<.05)&&turned==false && turncommand==false){
          tsetpoint=-200;
          turned=true;
        }
        if (turned==true && tsensorPosition>=-166 && tsensorPosition<=-164 ){
          turncommand=true;
          turn=0;
          tsetpoint=tsensorPosition;
          dsetpoint=1.6;
        }

        if (doutputSpeed > .5){
          doutputSpeed=.5;
        }
        if (doutputSpeed<-.6){
          doutputSpeed=-.6;
        }

        if (dsetpoint==1.6 &&dsensorPosition<2){
          grabpiece2=true;
          Hand.hsetpoint=-20;
          pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        }
        if (grabpiece2==true&&Hand.hande.getPosition()<-5){
          wrist.Wsetpoint=3.8;
        }



        
        Double tsensorPosition = drivetrain.m_gyro.getAngle();

        // calculations
       double terror = tsetpoint-tsensorPosition;
        if (Math.abs(terror) < tiLimit) {
       terrorSum += terror * dt;
        }

        double terrorRate = (terror - tlastError) / dt;

         turn = tkP * terror + tkI * terrorSum + tkD * terrorRate;
       
        // if (((Math.abs(tsetpoint-tsensorPosition))<.05)){
        // turn=0;
        // }
        // if (tsetpoint==-126){
        //   if (turn < -.45){
        //     turn=-.45;
        //   }
        //   if (turn > -.4){
        //     turn=-.4;
        //   }
        // }
        if (tsetpoint==0){
          turn=0;
        }

        // if (turn > .5){
        //   doutputSpeed=.5;
        // }
        // if (doutputSpeed<-.5){
        //   doutputSpeed=-.5;
        // }
        // update last- variables
        lastTimestamp = Timer.getFPGATimestamp();
        tlastError = terror;
        if (tsetpoint-tsensorPosition>-42&&tsetpoint<0){
          drivetrain.tdrive(doutputSpeed,doutputSpeed, false);
          dsetpoint=1.6;
        }
        else{
          drivetrain.tdrive(-turn+doutputSpeed,turn+doutputSpeed, false);
        }
        
          break;
        
        





          case kDefaultAuto:
          default:
          SmartDashboard.getBoolean("wait",waited);
           dt = Timer.getFPGATimestamp() - lastTimestamp;
          
          if (try_balancing==false){
            
            
            // Hand.hsetpoint=-20;
           // pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
          
           if (start_position==false){
            
            elbow.Esetpoint=-39.071121;
            shoulder.Ssetpoint = 150.377;
            elbow.EkP=0.05;
            start_position=true;
          }
           // calculations
            derror = dsetpoint - dsensorPosition;
           if (Math.abs(derror) < aiLimit) {
            derrorSum += derror * dt;
           }
   
            derrorRate = (derror - dlastError) / dt;
   
            doutputSpeed = (akP * derror + akI * derrorSum + akD * derrorRate);

            if (dsetpoint==0){
              if (doutputSpeed < -.6){
                doutputSpeed=-.6;
              }
              if (doutputSpeed>-.2){
                doutputSpeed=-.2;
              }
            }
           //drivetrain.tankDrive(doutputSpeed,doutputSpeed,false);
          //  drivetrain.mywatchdog();
          //  SmartDashboard.getNumber("output", doutputSpeed);
           // output to motors
          // double Speedvar = doutputSpeed;
           // update last- variables
           lastTimestamp = Timer.getFPGATimestamp();
           dlastError = derror;
           
           if (((Math.abs(dsetpoint-dsensorPosition))<.05)){
             placed=true;
             doutputSpeed =0;
            //  derrorSum = 0;
            //  lastTimestamp = 0;
            //  dlastError = 0;
            }
            // drivetrain.tdrive(doutputSpeed, doutputSpeed,false);
          
          if (placed == true && elbow.Elbowencoder.getPosition()<-30){
             wrist.Wsetpoint=-20;
            placed=false;
            place2=true;
          }

          if (wrist.wriste.getPosition() < -13 && place2==true ){
            waited=true;
            place2=false;
           
          }
          if (waited==true){
            Hand.hsetpoint = 0;
            pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
          }
          if (waited==true && Hand.hande.getPosition()> -3 ){
            wrist.Wsetpoint=0;
            shoulder.Ssetpoint=0;
            waited=false;
            akP=.15;
            dsetpoint=2;
            waited2=true;
          }
          if (waited2==true && dsensorPosition>.5 && dsensorPosition<.5){
            akP=.27;
          }
         
          if (waited2==true && dsensorPosition > .35 && fianl_driveup==false) {
          elbow.EkP=0.015;
          elbow.Esetpoint = 0;
          shoulder.Ssetpoint=0;
          }
          if (waited2==true && dsensorPosition > .2 && fianl_driveup==false && elbow.Elbowencoder.getPosition() >- 2) {
            dsetpoint = 4.2;
            }
          if (waited2==true && dsensorPosition>3.5 && dsensorPosition<3.9){
            fianl_driveup=true;
          }
          if (fianl_driveup==true){
            drivetrain.setbrake(false);
            akP=.25;
            dsetpoint = 1.6;
          }
        if (waited2==true && Math.abs(dsensorPosition-dsetpoint)<.2 && floppywrist==false &&fianl_driveup==true){
        //  dsetpoint=60;
          taxied=true;
          waited2=false;
          floppywrist=true;
          onchargestation=true;
          try_balancing=true;
          doutputSpeed=0;
        }
        // if (taxied==true &&Math.abs(dsensorPosition-dsetpoint)<.1){
        //   onchargestation=true;
        //   try_balancing=true;

        // }   
        
        }
        if (onchargestation==true && fianl_driveup==true){
      if (drivetrain.m_gyro.getYComplementaryAngle()<3 && drivetrain.m_gyro.getYComplementaryAngle()>-3){
        //chargestationbalance=true;
        Speedvar=0;
        drivetrain.setbrake(false);
      }
    
        else {
              setpoint = 0;
     
            // get sensor position
             sensorPosition = drivetrain.m_gyro.getYComplementaryAngle();
    
            // calculations
            berror = setpoint - sensorPosition;
            
            if (Math.abs(berror) < biLimit) {
              errorSum += berror * dt;
            }
    
            errorRate = (berror - lastError) / dt;
    
            doutputSpeed = bkP * berror + bkI * errorSum + bkD * errorRate;
    
            // output to motor
    
            // update last- variables
            lastTimestamp = Timer.getFPGATimestamp();
            lastError = berror;
          }
        }
          
          
            // if (drivetrain.m_gyro.getAngle()>3){
            //   turnerror = .05;
            //   }
            //   else if (drivetrain.m_gyro.getAngle()<2.5 && drivetrain.m_gyro.getAngle() >-2.5){
            //   turnerror =0;
            //   }
            //   else if (drivetrain.m_gyro.getAngle()<-3){
            //     turnerror =-.05;
            // }
            tsetpoint=0;
           tsensorPosition = drivetrain.m_gyro.getAngle();
    
            // calculations
            terror = tsetpoint-tsensorPosition;
            if (Math.abs(terror) < tiLimit) {
           terrorSum += terror * dt;
            }
    
             terrorRate = (terror - tlastError) / dt;
    
             turn = tkP * terror + tkI * terrorSum + tkD * terrorRate;
    
    
            // update last- variables
            lastTimestamp = Timer.getFPGATimestamp();
            tlastError = terror;
            
            if (Speedvar>.2){
              Speedvar=.2;
              }
              if (Speedvar<-.2){
                Speedvar=-.2;}
                
               double directionL= Speedvar;
               double directionR= Speedvar;
               
               if (doutputSpeed > .6){
                  doutputSpeed=.6;
                }
                if (doutputSpeed<-.6){
                  doutputSpeed=-.6;
                }
                drivetrain.tdrive(-turn+doutputSpeed,turn+doutputSpeed, false);
              // drivetrain.mywatchdog();

        break;
      
    
    }
  }




@Override
public void teleopInit(){
drivetrain.setbrake(true);
} 

@Override
  public void teleopPeriodic() {
    DataLogManager.start();
    
    //drivetrain.setbrake(false);
    speed_selected = speed_chooser.getSelected();
                SmartDashboard.putString("Speed Chosen", speed_selected);


                if (speed_selected == kDefaultSpeed) {
                        MaxDriveSpeed = 0.3;
                        speedMult = .4;
                } else {
                        MaxDriveSpeed = 0.6;
                        speedMult= .6;
                }


                Update_Limelight_Tracking();


                if (autoTargeting){
                        SmartDashboard.putString("autotargeting", "true");
                }else {
                        SmartDashboard.putString("autotargeting", "false");
            }


                if (targetSighted){
                        SmartDashboard.putString("targetSighted", "true");
                }else {
                        SmartDashboard.putString("targetSighted", "false");
            }


                if (targetAimed){
                        SmartDashboard.putString("targetAimed", "true");
                }else {
                        SmartDashboard.putString("targetAimed", "false");
            }


                if (targetInRange){
                        SmartDashboard.putString("targetInRange", "true");
                }else {
                        SmartDashboard.putString("targetInRange", "false");
            }


                if (targetLocked){
                        SmartDashboard.putString("targetLocked", "true");
                }else {
                        SmartDashboard.putString("targetLocked", "false");
            }
    
      
      if (controller2.getXButton()){
          setpoint = 0;
     
            // get sensor position
            Double sensorPosition = drivetrain.m_gyro.getYComplementaryAngle();
    
            // calculations
            berror = setpoint -0; //sensorPosition;
            double dt = Timer.getFPGATimestamp() - lastTimestamp;
    
            if (Math.abs(berror) < biLimit) {
              errorSum += berror * dt;
            }
    
            errorRate = (berror - lastError) / dt;
    
            Double outputSpeed = bkP * berror + bkI * errorSum + bkD * errorRate;
    
            // output to motors
            Speedvar=outputSpeed;
    
            // update last- variables
            lastTimestamp = Timer.getFPGATimestamp();
            lastError = berror;
            
          
            if (Speedvar>.2){
              Speedvar=.2;
              }
            if (Speedvar<-.2){
                Speedvar=-.2;}
    
               double directionL= Speedvar;
               double directionR= Speedvar;
            }
          else if(left.getTrigger()){
            drivetrain.arcadeDrive(left.getY()*speedMult,right.getX()*speedMult, false);
          }
        else if (right.getTrigger()){
          if (autoTargeting) {
                  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);        // turns off the limelight        
          } else {
                  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);        // turns on the limelight        
          }
          autoTargeting = !autoTargeting;
  } 


  if (autoTargeting && !targetSighted) {  // no target in sight so rotate and look for one.  this means that you cannot operator drive if the robot is in autoTargeting mode
          if (speed_selected == "Demo"){
                 // drivetrain.tdrive(0.2,-0.2);
          } else {
                  //drivetrain.tdrive(0.3,-0.3);                                                
          }
          targetAimed = false;
          targetInRange = false;
          //targetLocked = false;
          drivetrain.mywatchdog(); 
  }


  if (autoTargeting && targetSighted && !targetLocked) {  // drive closer to target
          double distance_adjust = .35;
          double heading_error = -tx;
          double distance_error = -ty;
          double steering_adjust_right; //= 0.05;
          double steering_adjust_left; //= 0.05;
          double left_command = 0.0;
          double right_command = 0.0;
          double distanceToTravel = 0.0;


          targetAimed = false;
          if (tx > 1.0) {
                  //steering_adjust_left = 0.04;  //aimAdjust*heading_error - min_aim_command;
                  //steering_adjust_left = ((0.3*heading_error/26.0) - (0.3/26.0))/2.0;  
                  steering_adjust_left = ((MaxTurnSpeed - MinTurnSpeed) * (heading_error - MinTurnDegrees) / (MaxTurnDegrees - MinTurnDegrees)) + MinTurnSpeed; // adjust left drive motor based on the tx value
                  steering_adjust_right = 0.0;
          }
          else if (tx < -1.0) {
                  //steering_adjust_right = 0.04;  //aimAdjust*heading_error + min_aim_command;
                  steering_adjust_left = 0.0;
                  //steering_adjust_right = ((0.3 * heading_error / 26.0) - (0.3 / 26.0))/2.0;  
                  steering_adjust_right = ((MaxTurnSpeed - MinTurnSpeed) * (heading_error - MinTurnDegrees) / (MaxTurnDegrees - MinTurnDegrees)) + MinTurnSpeed; // adjust right drive motor based on the tx value
          } else {
                  steering_adjust_right = 0.0;
                  steering_adjust_left = 0.0;
                  targetAimed = true;
          }
          //steering_adjust = 0.0;
          
          //targetAimed = true;
          //targetInRange = false;
          
          distanceToTravel = Math.abs(distanceToTarget - TargetCenter);
          //distance_adjust = (0.4* distanceToTravel / 119.0) - (0.4 / 119.0) + 0.3;
          distance_adjust = ((MaxDriveSpeed - MinDriveSpeed) * (distanceToTravel - MinDriveDistance) / (MaxDriveDistance - MinDriveDistance)) + MinDriveSpeed; // set motor drive speed based on our distance away from target
          
          if ((distanceToTarget >= MinTargetRange) && (distanceToTarget < MaxTargetRange)) {
                  left_command = 0.0;
                  right_command = 0.0;
                  targetInRange = true;
          } else if (distanceToTarget < MinTargetRange) {
                  left_command = -steering_adjust_left - distance_adjust;
                  right_command = steering_adjust_right - distance_adjust;
          } else if (distanceToTarget >= MaxTargetRange) {
                  left_command = -steering_adjust_left + distance_adjust;
                  right_command = steering_adjust_right + distance_adjust;
          }


          SmartDashboard.putNumber("left_drive_speed", left_command);
          SmartDashboard.putNumber("right_drive_speed", right_command);
          SmartDashboard.putNumber("distanceToTravel", distanceToTravel);
                                  
          //targetLocked = false;
          if ((targetAimed) && (targetInRange)) {
                  targetLocked = true;
                  drivetrain.tankDrive(0.0,0.0,true);
                  autoTargeting = !autoTargeting;
          }
          else {
            //drivetrain.mywatchdog();
            //drivetrain.tdrive(left_command, right_command);
                  //drivetrain.mywatchdog();
          }
  } 
  else {
    if (!autoTargeting || (autoTargeting && !targetSighted)) {
      if (!autoTargeting) {  //if we are not auto-targeting, allow drive to work as normal
              targetAimed = false;
              targetInRange = false;
              targetLocked = false;
              drivetrain.tankDrive(right.getY() * speedMult, left.getY() * speedMult,false);
              drivetrain.mywatchdog(); 
  }
}
  }
  
  if (autoTargeting && targetSighted && targetLocked) {
          //uptake1.set(.5);
          
          //autoTargeting = false;
  }
          
            
            
      





          
      // // Hand controlled by left and right triggers
      if (controller2.getXButton()) {
        Hand.hsetpoint = 0;
      pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      } 
        else if (controller2.getYButton()) {
          Hand.hsetpoint=-20;
    pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
      }
    
    if (controller2.getAButton()) {
      elbow.Esetpoint=-23;
      elbow.EkP=0.05;
      } 
     if (controller2.getRightTriggerAxis()>.1) {
      wrist.Wsetpoint=0;
      elbow.EkP=0.005;
      elbow.Esetpoint = 0;
      shoulder.Ssetpoint=0;
    }
    if (controller2.getBButton()){
      elbow.Esetpoint=-28; 
      elbow.EkP=0.05;
      //drivetrain.m_gyro.reset();
      
    }
   
    if (controller2.getPOV()==0){
      wrist.Wsetpoint= 3.8;
    }

        if (controller2.getBackButton()){
         // drivetrain.m_gyro.reset();
        }
      
       //high cone
       if (controller2.getLeftTriggerAxis()>.1) {
          elbow.Esetpoint=-39.071121;
          shoulder.Ssetpoint = 150.377;
          elbow.EkP=0.05;
        } 
         
         
       
       if(controller2.getPOV()==180){
         wrist.Wsetpoint=0;}
         else if (controller2.getLeftBumper()){
         wrist.Wsetpoint=-20;  
       }

}








public void Update_Limelight_Tracking() {
  tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);  // 0 or 1
  tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);  // -27 to 27
  ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);  // -20.5 to 20.5
  ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);  // 0 to 100


  SmartDashboard.putNumber("LimelightX", tx);
  SmartDashboard.putNumber("LimelightY", ty);
  SmartDashboard.putNumber("LimelightArea", ta);
  SmartDashboard.putNumber("TargetSpotted", tv);


  if (tv < 1.0) {
          targetSighted = false;
          distanceToTarget = -1.0;
  } else {
          targetSighted = true;        


          //calculate distance to target
          double angleToTargetAsRadians = (limelightAngle + ty) * (3.14159 / 180.0);  
          distanceToTarget = (targetHeight - limelightHeight)/Math.tan(angleToTargetAsRadians);
  }


  SmartDashboard.putNumber("Distance", distanceToTarget);
  return;
}

}