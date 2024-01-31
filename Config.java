package org.firstinspires.ftc.teamcode;


public class Config {
  /*****************************************************************************
  ** HARDWARE CONSTANTS
  *****************************************************************************/
  public static final String FRONT_RIGHT_WHEEL_NAME = "m3";
  public static final String FRONT_LEFT_WHEEL_NAME = "m2";
  public static final String BACK_LEFT_WHEEL_NAME = "m1";
  public static final String BACK_RIGHT_WHEEL_NAME = "m4";
  
  public static final String SHOULDER_NAME = "shoulder";
  public static final String SLIDE_NAME = "lift";
  
  public static final String CLAW_LEFT_NAME = "clawLeft";
  public static final String CLAW_RIGHT_NAME = "clawRight";
  public static final String WRIST_NAME = "wrist";
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DISTANCE CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int TICKS_PER_360_DEG = 4475;
  public static final int TILE_LENGTH = 1250;

  public static final int DEFAULT_SPIKE_MARK = 3;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DRIVE SPEED CONSTANTS
  *****************************************************************************/
  public static final int CRUISE_SPEED = 1400;
  public static final int ARM_VELOCITY = 1250;
  public static final int SLIDE_VELOCITY = 1200;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** ARM CALIBRATION CONSTANTS
  *****************************************************************************/
  //public static final int BOARD_HEIGHT = 70;
  
  //public static final int BOTTOM = 0;
  //public static final int CRUISING_HEIGHT = 30;
  //public static final int TOP = 125;
  // ---------------------------------------------------------------------------
  
  
  /*****************************************************************************
  ** CLAW CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final double CLAW_LEFT_OPEN = .2; 
  public static final double CLAW_LEFT_CLOSE = .62; 
  public static final double CLAW_RIGHT_OPEN = .8;
  public static final double CLAW_RIGHT_CLOSE = .38;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** CAMERA CONSTANTS
  *****************************************************************************/
  public static final String CAMERA_NAME = "Webcam 1";
	public static final int CAMERA_RESO_X = 640;
	public static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/teampiece.tflite";
	public static final String[] LABELS = { // Define the labels used in our model (must be in training order!)
		"BLUE",
		"RED",
	};
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** CAMERA CALIBRATION CONSTANTS
  * Lens intrinsics
  * UNITS ARE PIXELS
  * NOTE: this calibration is for the C920 webcam at 800x448.
  * You will need to do your own calibration for other configurations!

  Resolution: 1280x720
  Pixel Size: 2.8um
  Sensor Size: 3.58x2.02mm
  Stock lens focal length: 4.2mm
  *****************************************************************************/
  public static final double FX = 1430;
  public static final double FY = 1430;
  public static final double CX = 480;
  public static final double CY = 620;

  public static final double TAGSIZE = 0.166;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DETECTION CONSTANTS
  *****************************************************************************/
  public static final float DECIMATION_HIGH = 3;
  public static final float DECIMATION_LOW = 2;
  public static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
  public static final int NUM_FRAMES_BEFORE_LOW_DECIMATION = 4;
  public static final int MAX_NUM_FRAMES_NO_DETECTION = 100; // How many attempts to detect before giving up
  // ---------------------------------------------------------------------------
}