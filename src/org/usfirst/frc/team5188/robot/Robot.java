
package org.usfirst.frc.team5188.robot;

import java.util.TimerTask;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import modules.Pats_PID_Controller;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends IterativeRobot {
	
	final String defaultAuto = "default";
	final String gear = "gear";
	final String baseline = "baseline";
	final String gearTime = "gearTime";
	int counter = 0;
	String auto;
	double error = 0;
	
	Encoder lEncoder = new Encoder(2,3);

	
//	M_I2C i2c = new M_I2C();
	AHRS gyro = new AHRS(SerialPort.Port.kMXP);
//	PixySensor pidPixy = new PixySensor(i2c);
	
	DriverStation driverStation = DriverStation.getInstance();
	
	Drive drive = new Drive(0, 1);
	Auto autoFunc = new Auto(drive, gyro, lEncoder);


	
	GyroSensorActuator pidGyro = new GyroSensorActuator(drive, gyro);
	SendableChooser<String> chooser = new SendableChooser<>();
	
//	Pats_PID_Controller pixyPid = new Pats_PID_Controller(0.5, .006 , .0006 , 10, pidPixy, pidGyro);


	Timer time = new Timer();
	boolean base = false, timer = false;
	
	
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("Gear", gear);
		chooser.addObject("Base Line", baseline);		
		chooser.addObject("Time", gearTime);

		
		SmartDashboard.putData("Auto Choices", chooser);

		lEncoder.setDistancePerPulse(0.08590292414);
		lEncoder.reset();
		gyro.reset();

		
	}
	@Override
	public void autonomousInit() {
//		lEncoder.reset();
		base = false;
		timer = false;
		auto = chooser.getSelected();
	}
	
	
	
	
	@Override
	public void autonomousPeriodic() {
		auto = chooser.getSelected();
		switch (auto){
		
		case gearTime:
			while(driverStation.isAutonomous()){
				while (!timer){
					drive.setLDrive(0.496);
					drive.setRDrive(0.5);
					
//					time.schedule(new TimerTask(){
//					@Override	
//					public void run() {
//						timer = true;
//					}
//					}, 2000);
				
				}
				drive.stop();
			}
			break;
		
		case baseline:
//
//			while (!timer){
//				drive.setLDrive(0.496);
//				drive.setRDrive(0.5);
//				
//				time.schedule(new TimerTask(){
//				@Override	
//				public void run() {
//					timer = true;
//				}
//				}, 2500);
//			
//			}
//			drive.stop();
			
			
			break;
			
		case gear:
		
			break;
			
		default:
			drive.setLDrive(0);
			drive.setRDrive(0);
		}
	}
		


	@Override
	public void teleopPeriodic() {
		if (Control.drive.isButtonPushed(CTRL_BTN.A)){
			autoFunc.DriveStraightFor(36);
			Timer.delay(1);
			autoFunc.RotateToDegreesIteration(-60, 1, .3);
			Timer.delay(1);
			autoFunc.DriveStraightFor(36);
			
			
		}
		if (Control.drive.isButtonPushed(CTRL_BTN.Y)){
			autoFunc.DriveStraightFor(36);

		}
		if (Control.drive.isButtonPushed(CTRL_BTN.X)){
			autoFunc.DriveStraightFor(36);
			Timer.delay(1);
			autoFunc.RotateToDegreesIteration(60, .5, .3);
			Timer.delay(1);
			autoFunc.DriveStraightFor(36);
		}
		
		if (Control.drive.isButtonPushed(CTRL_BTN.B)){
		gyro.reset();
		resetEncoders();
		}
		

		double throttle = -Control.drive.get(CTRL_AXIS.LY);
		double turn = Control.drive.get(CTRL_AXIS.RX);
		double shifter = Control.drive.isButtonHeld(CTRL_BTN.R) ? 0.5 : 1.0;
		
		if (Control.drive.isButtonHeld(CTRL_BTN.L)){
			drive.setRDrive(-Control.drive.get(CTRL_AXIS.RY)*shifter);
			drive.setLDrive(-Control.drive.get(CTRL_AXIS.LY)*shifter);
		}else{

			drive.setRDrive(throttle * shifter * (1 - Math.max(0, turn)));
			drive.setLDrive(throttle * shifter * (1 + Math.min(0, turn)));
		
		}

	}
	
	
	public void resetEncoders(){
		lEncoder.reset();
	}

//	private void visionTrack(){
//		PixyPacket pkt = i2c.getPixy();
//		if(pkt.x != -1){
//			if(pkt.x < .48 || pkt.x > .52){//Only start PID if off centered
//				pixyPid.set(0.5);
//				pixyPid.start();
//				while(pkt.x < .48 || pkt.x > .52){
//					if (!Control.drive.isButtonHeld(CTRL_BTN.Y))
//						break;
//					if(pkt.x == -1)//Restart teleop if ball lost during turn
//						break;
//					
//					pkt = i2c.getPixy();
//					System.out.println("XPos: " + pkt.x);
//				}
//				pixyPid.stop();
//			}
//			if(pkt.area <= 0.05 && pkt.area > 0){
//				drive.setLDrive(0.3);
//				drive.setRDrive(0.3);
//				System.out.println("Area: " + pkt.area);
//			}else if(pkt.area >= 0.9){
//				drive.setLDrive(-0.3);
//				drive.setRDrive(-0.3);
//				System.out.println("Area: " + pkt.area);
//
//			}else{
//				drive.stop();
//				System.out.println("Area: " + pkt.area);
//
//			}
//			
//		}else{//Dont move if see nothing
//			drive.stop();
//		}
//	}
		
	
	@Override
	public void testPeriodic() {

	}
}
