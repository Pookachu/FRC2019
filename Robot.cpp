/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/* Discord Hypesquad house of balance FTW                                     */
/* Made by Ethan (2019).                                                      */
/* For Support watch the documentation video at                               */
/* https://www.youtube.com/watch?v=dQw4w9WgXcQ                                */
/*----------------------------------------------------------------------------*/
#include <thread>

#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <wpi/raw_ostream.h>
#include <AHRS.h>
#include <AHRSProtocol.h>
#include <frc/WPILib.h>

class Robot : public frc::TimedRobot
{
//#if defined(__linux__)
private:
	//Static PI declaration
	static constexpr double PI = 3.14159265359;

	//Static PWM port declarations
	static constexpr int
			PWMZero = 0,
			PWMOne = 1,
			PWMTwo = 2,
			PWMThree = 3,
			PWMFour = 4,
			PWMFive = 5, 
			PWMSix = 6,
			PWMSeven = 7,
			PWMEight = 8,
			PWMNine = 9,

			//Static DIO port declarations
			DIOZero = 0,
			DIOOne = 1,
			DIOTwo = 2,
			DIOThree = 3,
			DIOFour = 4,
			DIOFive = 5,
			DIOSix = 6,
			DIOSeven = 7,
			DIOEight = 8,
			DIONine = 9,

			//Static Analog port declarations
			AnalogZero = 0,
			AnalogOne = 1,
			AnalogTwo = 2,
			AnalogThree = 3,

			//Static Relay port declarations
			RelayZero = 0,
			RelayOne = 1,
			RelayTwo = 2,
			RelayThree = 3,

			//Static Can port declarations
			CanZero = 0,
			CanOne = 1,
			CanTwo = 2,
			CanThree = 3,

			//Static lift positon "variables"
			//ball heights
			Lpos1 = 62,
			Lpos2 = 210,
			Lpos3 = 350,
			//panel heights
			Lpos4 = 80,
			Lpos5 = 230,
			Lpos6 = 365;

	static constexpr double hover = 0.05;
	double CurrentLift;

	//Joystick declaration
	frc::Joystick m_stick{0};
	frc::Joystick m_stick2{1};

	//Drive motor declarations
	/*
	0/| |\2
	1\| |/3
	*/
	frc::Spark m_frontLeft{PWMZero};
	frc::Spark m_rearLeft{PWMOne};
	frc::Spark m_frontRight{PWMTwo};
	frc::Spark m_rearRight{PWMThree};

	//Initiating robot drive at m_drive
	frc::MecanumDrive m_drive{m_frontLeft, m_rearLeft, m_frontRight, m_rearRight};

	//Solenoid declaration (CAN)
	frc::Solenoid m_solenoidFront{4};
	frc::Solenoid m_solenoidBack{7};

	//Other motor definitions
	//Lift
	frc::PWMVictorSPX m_lift{PWMFour};

	//Kick
	//frc::PWMTalonSRX m_kick{PWMEight};

	//Tilt
	frc::Spark m_tilt{PWMFive};

	//Grab
	frc::Spark m_grab{PWMEight};

	//Servo init
	Servo *m_servo = new Servo(PWMSeven);

	//Sensor definitions
	//Ultrasonic initialization
	//AnalogInput *m_Ultrasonic = new AnalogInput(AnalogZero);

	//Gyroscope initialization
	AHRS *m_ahrs = new AHRS(SPI::Port::kMXP);

	/*ai = new AnalogInput(0);*/

	//Counter init
	Encoder *m_encoder = new Encoder(0, 1, true, Encoder::EncodingType::k4X);

	//Limit switch init
	DigitalInput *m_topLimit = new DigitalInput(3);
	DigitalInput *m_bottomLimit = new DigitalInput(6);
	DigitalInput *m_climbLimitFront = new DigitalInput(4);
	DigitalInput *m_climbLimitBack = new DigitalInput(5);

	//Misc Declarations
	//Timer init
	frc::Timer m_timer;
	frc::Timer m_kicktimer;
	frc::Timer m_climbtimer;

	//LiveWindow init
	//frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
public:
	//Ultrasonic calculation variables
	//double ultraCal = 3.25;
	//double distToWall;
	//counter variables
	//	double diameter = 6/12; // 6 inch wheels
	double dist = 0.5 * 3.14 / 1024; // ft per pulse
	bool Climb = 0;
	double target = 0;
	//Robot init function
	bool started = 0;

	double LiftSmoothSpeedUp = .1;
	double LiftSmoothSpeedDown = 0;
	bool AdjustUp;
	bool AdjustDown;

	double LiftMiddle = 0;

	void RobotInit() override
	{
		//Start the timer
		m_timer.Start();

		//Counter settings
		m_encoder->SetDistancePerPulse(dist);

		/*
		Safeties
		*/
		m_drive.SetSafetyEnabled(false);
		//drive expiration? check later
		m_drive.SetExpiration(0.1);

//Start VisionThread in a seperate thread
#if defined(__linux__)
		std::thread visionThread(VisionThread);
		visionThread.detach();
#else
		wpi::errs() << "Vision only available on Linux.\n";
		wpi::errs().flush();
#endif
	}

	//AUTONOMOUS FUNCTIONS
	void AutonomousInit() override
	{
		Climb = 0;
		//Timer resets before we start autonomous
		m_timer.Reset();
		m_timer.Start();

		//Encoder resets before we start autonomous
		m_encoder->Reset();
	}
	void AutonomousPeriodic() override
	{
		if (m_timer.Get() <= .5)
		{
			m_tilt.Set(-.2);
		}
		else if(m_timer.Get() < 1 && m_timer.Get() > 1.5)
		{
			m_tilt.Set(0);
		}
		if (m_timer.Get() <= 2)
		{
			if (!(m_bottomLimit->Get()))
			{
				m_lift.Set(0);
			}
			else
			{
				m_encoder->Reset();
			}
		}
		else
		{
			Working();
		}
	}

	void TeleopPeriodic() override
	{
		//Init once (Pragma haha)
		if (started == false)
		{
			Climb = 0;
			started = true;
			//Counter variable declaration
			target = 0;
			m_kicktimer.Start();
			m_encoder->Reset();
		}
		Working();
	}

	void DisabledInit() override
	{
		target = 0;
		Climb = false;
	}
	void Working()
	{
		//DEPRECATED
		/*
		Ultrasonic code
		*/
		//Trying to calculate the ultrasonic sensor value in mm
		//distToWall = (m_Ultrasonic->GetValue()/ultraCal);

		//DISABLED TEMPORARILY
		/*
		Solenoid Control Declaration
		*/
		/*
		Debugging
		*/
		//Joystick HAT testing
		SmartDashboard::PutNumber("POV test", m_stick.GetPOV());

		//Raw Counter Info
		SmartDashboard::PutNumber("Encoder Ticks", m_encoder->Get());

		//Processed Counter info
		SmartDashboard::PutNumber("Distance", m_encoder->GetDistance());

		//Processed Ultrasonic info
		//SmartDashboard::PutNumber("Distance to wall", distToWall);

		SmartDashboard::PutBoolean("Limit switch top", m_topLimit->Get());
		SmartDashboard::PutBoolean("Limit switch bottom", m_bottomLimit->Get());
		SmartDashboard::PutBoolean("Limit switch climb front", m_climbLimitFront->Get());
		SmartDashboard::PutBoolean("Limit switch climb back", m_climbLimitBack->Get());
		/*
		Seperate Functions for organization and simplicity (Declared below)
		*/

		Drive();

		Tilt();

		//if(m_timer.Get() < 135)
		//																													{
		//Kick();
		//																													}
		//else
		//																													{
		Solenoids(m_climbLimitFront->Get(), m_climbLimitBack->Get(), m_stick2.GetRawButton(1));
		//																													}
		Ball();

		//SonicCalibration();

		CurrentLift = m_encoder->GetDistance();

		LiftButtonGet();
		LiftMovementLogic();
		LiftManualAdjust();
	}

	//DEPRECATED
	//navX Reset Function
	/*	void NavXReset()
	{
		//Gyroscope reset
		bool reset_yaw_button_pressed = m_stick.GetRawButton(1);
		if (reset_yaw_button_pressed)
		{
			m_ahrs->ZeroYaw();
		}
	} */
	void Solenoids(bool ClimbLimitFront, bool ClimbLimitBack, bool ClimbButton)
	{
		//State Variables
		static bool
			FrontUp = true,
			Drive = false,
			BackUp = true,
			FrontPressed = false,
			BackPressed = false;

		static int
			FrontCount = 0,
			BackCount = 0;

		//State Setup
		if (FrontUp)
		{
			m_solenoidFront.Set(false);
		}
		else if ((!(FrontUp)))
		{
			m_solenoidFront.Set(true);
		}

		if (BackUp)
		{
			m_solenoidBack.Set(false);
		}
		else if ((!(BackUp)))
		{
			m_solenoidBack.Set(true);
		}

		if (ClimbButton)
		{
			target = 0;
			Climb = true;
			m_climbtimer.Start();
			BackUp = false;
			FrontUp = false;
		}
		else
		{
			Climb = false;
			BackUp = true;
			FrontUp = true;
		}

		if (Drive && Climb)
		{
			m_drive.DriveCartesian(0, .25, 0);
		}

		//Real Sequence
		if (ClimbLimitFront && Climb)
		{
			FrontPressed = true;
			if (FrontCount > 0)
			{
				FrontUp = true;
			}
		}
		else if(FrontPressed && Climb)
		{
			FrontCount++;
			FrontPressed = false;
		}

		if (ClimbLimitBack && Climb)
		{
			BackPressed = true;
			if (BackCount > 0)
			{
				BackUp = true;
			}
		}
		else if(BackPressed && Climb)
		{
			BackCount++;
			BackPressed = false;
		}

		if (m_climbtimer.Get() > 10)
		{
			Drive = false;
		}
	}

	void Drive()
	{
		//Assign variables for Joystick axii
		double driveX = m_stick.GetX();
		double driveY = m_stick.GetY();
		double driveZ = m_stick.GetZ();
		double driveThrottle = (((m_stick.GetThrottle() + 1) / 2) * -1);
		//Set deadzone
		double deadZone = 0.1;
		double magnitude = sqrt(driveX * driveX + driveY * driveY);

		driveX /= magnitude;
		driveY /= magnitude;

		if (magnitude < deadZone)
		{
			magnitude = 0; //no movement in deadzone radius
		}
		else
		{
			magnitude -= deadZone; //no discontinuity
		}

		driveX *= magnitude; //scale each amount
		driveY *= magnitude;


		if(m_stick.GetPOV() == -1)
		{
			m_drive.DriveCartesian(
				driveThrottle * (driveX * -1), //reversed
				driveThrottle * driveY,
				driveThrottle * -driveZ);
		}
		else if(m_stick.GetPOV() == 90)
		{
			m_drive.DriveCartesian(
				.75, 
				driveThrottle * driveY,
				driveThrottle * -driveZ);
		}
		else if(m_stick.GetPOV() == 270)
		{
			m_drive.DriveCartesian(
				-.75,
				driveThrottle * driveY,
				driveThrottle * -driveZ);
		}
	}

	//DEPRECATED
	//Ultra Sonic Calibration Function Declaration
	/*	void SonicCalibration() 
	{	
		if(m_stick.GetRawButton(9)) 
		{
			ultraCal = ultraCal +.01;
		}

		if(m_stick.GetRawButton(7)) 
		{
			ultraCal = ultraCal -.01;
		}
	}*/

	void Ball()
	{
		//Basic control scheme
		if (m_stick.GetRawButton(4) || m_stick.GetRawButton(1))
		{
			m_grab.Set(.5);
			return;
		}
		else if (m_stick.GetRawButton(6))
		{
			m_grab.Set(-.5);
			return;
		}
		else
		{
			m_grab.Set(0);
		}
	}

	void Tilt()
	{
		//Basic control scheme
		if (m_stick.GetRawButton(5))
		{
			m_tilt.Set(.7);
			return;
		}
		else if (m_stick.GetRawButton(3) 
			|| m_stick.GetRawButton(1) 
			|| m_stick.GetRawButton(4))
		{
			m_tilt.Set(-.5);
			return;
		}
		else
		{
			m_tilt.Set(0);
		}
	}

	void Kick()
	{
		
		/*static bool sequence = 0;
		if (m_stick.GetRawButton(1) && sequence == 0)
		{
			m_kicktimer.Start();
			m_kicktimer.Reset();
			sequence = 1;
		}
		if (m_kicktimer.Get() < .2 & sequence == 1)
		{
			m_kick.Set(1);
		}
		else if (m_kicktimer.Get() < .4 & sequence == 1)
		{
			m_kick.Set(-.5);
		}
		else if (m_kicktimer.Get() > .4 & sequence == 1)
		{
			m_kick.Set(0);
			sequence = 0;
			m_kicktimer.Stop();
		}
		SmartDashboard::PutBoolean("Kick Sequence", sequence);*/
	}

	// Fetch and give LiftTargetControl button pressed info
	void LiftButtonGet()
	{

		for (int i = 7; i <= 13; i++)
		{
			//This is a really dumb way to check the 2 button as well as 7-12
			if (i == 13)
			{
				i = 2;
			}
			//Main part of this function, checks buttons 2, and 7-12 to see if they're on and sends if they are to
			if (CheckButton(i))
			{
				LiftTargetControl(i);
			}
			//Set it back to not screw up the for loop
			if (i == 2)
			{
				i = 13;
			}
			SmartDashboard::PutBoolean("CheckButton", CheckButton(i));
		}
	}

	// Checking to see if a specific button is pressed to be able to implement a for loop
	bool CheckButton(int buttonNumber)
	{
		// Fetching state for button number
		if (m_stick.GetRawButton(buttonNumber))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	// Sets the target variable depending on which button was pressed
	void LiftTargetControl(int button)
	{
		// Assign button value to target
		switch (button)
		{
		case 7:
			target = Lpos3;
			break;
		case 8:
			target = Lpos6;
			break;
		case 9:
			target = Lpos2;
			break;
		case 10:
			target = Lpos5;
			break;
		case 11:
			target = Lpos1;
			break;
		case 12:
			target = Lpos4;
			break;
		default:
			target = 0;
			break;
		}
		LiftMiddle = (CurrentLift + target) / 2;
	}

	//The real control for where the lift is and where it should be
	void LiftMovementLogic()
	{
		// Debug variables
		bool ifHover = false;
		bool ifUp = false;
		bool ifDown = false;
		bool ifBottom = false;

		// Bottom (special case)
		// We want to go to the bottom but are not currently there
		if (target == 0 && (!(m_bottomLimit->Get())))
		{
			ifUp = false;
			ifDown = true;
			ifHover = false;
			ifBottom = true;

			LiftSmoothSpeedUp = .1;
			LiftSmoothSpeedDown = 0;
			m_lift.Set(0);
		}
		// Hover
		// if within 5 of target value, hover
		else if ((CurrentLift < target + 5) && (CurrentLift > target - 5))
		{
			ifUp = false;
			ifDown = false;
			ifHover = true;
			ifBottom = false;

			LiftSmoothSpeedUp = .1;
			LiftSmoothSpeedDown = 0;
			m_lift.Set(hover);
		}
		// Up
		// If where the lift is is lower than the target, go up
		else if (CurrentLift < target && m_topLimit->Get())
		{

			ifUp = true;
			ifDown = false;
			ifHover = false;
			ifBottom = false;

			LiftSmoothMovement(true);
			m_lift.Set(LiftSmoothSpeedUp);
		}

		// Down
		// If where the lift is is higher than the target, go down
		else if (CurrentLift > target && (!(m_bottomLimit->Get())))
		{
			ifUp = false;
			ifDown = true;
			ifHover = false;
			ifBottom = false;

			LiftSmoothMovement(false);
			m_lift.Set(LiftSmoothSpeedDown);
		}
		// Encoder reset when we hit the bottom of the lift
		if (m_bottomLimit->Get() == true)
		{
			m_encoder->Reset();
		}
		SmartDashboard::PutNumber("Target:", target);
		SmartDashboard::PutBoolean("Lift Up", ifUp);
		SmartDashboard::PutBoolean("Lift Down", ifDown);
		SmartDashboard::PutBoolean("Lift Hover", ifHover);
		SmartDashboard::PutNumber("Lift Rate", m_encoder->GetRate());
	}

	//Manual adjustments with the POV (Joystick hat)
	void LiftManualAdjust()
	{
		// Target + 5 if POV is up
		if (m_stick.GetPOV() == 0 && AdjustUp == false)
		{
			target = target + 5;
			LiftMiddle = (CurrentLift + target) / 2;
			AdjustUp = true;
			AdjustDown = false;
			return;
		}
		// POV middle resets other two to be able to be pressed again
		if (m_stick.GetPOV() == -1)
		{
			AdjustUp = false;
			AdjustDown = false;
			return;
		}
		// Target - 5 if POV is down
		if (m_stick.GetPOV() == 180 && AdjustDown == false)
		{
			target = target - 5;
			LiftMiddle = (CurrentLift + target) * .75;
			AdjustUp = false;
			AdjustDown = true;
		}
	}

	void LiftSmoothMovement(bool Direction)
	{
		double MaxSpeedUp = .8;
		double MinSpeedUp = .27;
		double MaxSpeedDown = -.3;
		double MinSpeedDown = 0;
		//up
		if (Direction)
		{
			//if where we on the lift is lower than between the start position of the lift and the target, and it hasn't hit the max up speed, increment.
			if ((CurrentLift < LiftMiddle) && LiftSmoothSpeedUp < MaxSpeedUp)
			{
				LiftSmoothSpeedUp = LiftSmoothSpeedUp + .02;
				return;
			}
			//if where we on the lift is higher than between the start position of the lift and the target, and it hasn't hit the max up speed, drecrement.
			else if ((CurrentLift > LiftMiddle) && LiftSmoothSpeedUp > MinSpeedUp)
			{
				LiftSmoothSpeedUp = LiftSmoothSpeedUp - .02;
			}
		}
		//down
		else
		{
			//if where we on the lift is higher than between the start position of the lift and the target, and it hasn't hit the max up speed, decrement.
			if ((CurrentLift > LiftMiddle) && LiftSmoothSpeedDown < MaxSpeedDown)
			{
				LiftSmoothSpeedDown = LiftSmoothSpeedDown - .02;
				return;
			}
			//if where we on the lift is lower than between the start position of the lift and the target, and it hasn't hit the max up speed, increment.
			else if ((CurrentLift < LiftMiddle) && LiftSmoothSpeedDown > MinSpeedDown)
			{
				LiftSmoothSpeedDown = LiftSmoothSpeedDown + .02;
			}
		}
	}

	//VisionThread Declaration
	static void VisionThread()
	{
		// Get the USB camera from CameraServer
		cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

		// Set the resolution
		camera.SetResolution(640, 480);

		// Get a CvSink. This will capture Mats from the Camera
		cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

		// Setup a CvSource. This will send images back to the Dashboard
		cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);

		// Mats are very memory expensive. Lets reuse this Mat.
		cv::Mat mat;

		while (true)
		{
			// Tell the CvSink to grab a frame from the camera and
			// put it
			// in the source mat.  If there is an error notify the
			// output.
			if (cvSink.GrabFrame(mat) == 0)
			{
				// Send the output the error.
				outputStream.NotifyError(cvSink.GetError());
				// skip the rest of the current iteration
				continue;
			}
			// Put a rectangle on the image
			rectangle(mat, cv::Point(330, 30), cv::Point(310, 260), cv::Scalar(255, 255, 255), 5);
			// Give the output stream a new image to display
			outputStream.PutFrame(mat);
		}
	}
//#endif
};
#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
// discordapp.com/invite/GFgEJKN
