package application;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Properties;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.RadioButton;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleGroup;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import utils.Utils;

public class CC_Controller
{
	// =====[Private] FXML Components=====
	@FXML
	private Button connCamera1;
	@FXML
	private Button connCamera2;
	@FXML
	private Button selectFileButton;
	@FXML
	private Button detectButton;
	@FXML
	private Button calibrateButton;
	@FXML
	private Button undistButton;
	@FXML
	private Button recordButton;
	@FXML
	private Button saveParamButton;
	@FXML
	private Button solveButton;
	@FXML
	private Button closeButton;
	@FXML
	private Button testMark;
	@FXML
	private CheckBox checkUseParam;
	@FXML
	private RadioButton radioRM;
	@FXML
	private RadioButton radioFM;
	@FXML
	private ToggleGroup radioGroup_CM;
	/**
	 * The FXML area for showing the current frame (before calibration)
	 */
	@FXML
	private ImageView originalFrame;
	/**
	 * The FXML area for showing the current frame (after calibration)
	 */
	@FXML
	private ImageView calibrateFrame;
	@FXML
	private Label originalLabel;
	@FXML
	private Label procStreamLabel;
	@FXML
	private TextField numBoards;
	@FXML
	private TextField numSaved;
	@FXML
	private TextField numHorCorners;
	@FXML
	private TextField numVertCorners;
	@FXML
	private TextField lengthHorCorners;
	@FXML
	private TextField lengthVertCorners;
	@FXML
	private TextArea messageArea;
	@FXML
	private Label recordCount;
	@FXML
	private TextField clickX;
	@FXML
	private TextField clickY;
	@FXML
	private TextField worldX;
	@FXML
	private TextField worldY;
	@FXML
	private TextField worldZ;
	@FXML
	private TextField testX;
	@FXML
	private TextField testY;
	@FXML
	private TextField testZ;
	@FXML
	private VBox recordArea;
	@FXML
	private VBox testArea;
	
	
	// =====[Private] Variable=====
	
	/**
	 * A timer for acquiring the video stream, and taking snapshot
	 */
	private ScheduledExecutorService timer;
	/**
	 * The OpenCV object that performs the video capture
	 */
	private VideoCapture capture;
	/**
	 * A flag to change the button behavior
	 */
	private boolean cameraActive;
	/**
	 * The saved chessboard image
	 */
	private Mat savedImage;
	/**
	 * The calibrated camera frame
	 */
	private Image procImage, CamStream;
	/**
	 * The id of the camera to be used
	 */
	private int cameraId;
	/**
	 * A preview point that would be confirmed to record for calculating <br>
	 * the rotation and translation vectors (extrinsic parameters)
	 */
	private Point previewPoint;
	/**
	 * The points for calculating the rotation and translation vectors, <br>
	 * manually collecting them by mouse clicking in a captured image.
	 */
	private List<Point> testImgPoints;
	/**
	 * The points for calculating the rotation and translation vectors, <br>
	 * manually input the position in the form of real world coordinate.
	 */
	private List<Point3> testObjPoints;
	/**
	 * A point that is projected from a 3D testing point.
	 */
	private Point testPoint;
	/**
	 * The sample list of image's 2D points
	 */
	private List<Mat> imagePoints;
	/**
	 * The sample list of reference system's 3D points
	 */
	private List<Mat> objectPoints;
	/**
	 * The image's 2D points of the chessboard corners in pixel coordinates
	 */
	private MatOfPoint2f imageCorners;
	/**
	 * The reference system's 3D points where the chessboard is located in world coordinates
	 * (assume the Z axe is always 0)
	 */
	private MatOfPoint3f obj;
	/**
	 * Sample number of the chessboards
	 */
	private int boardsNumber;
	/**
	 * The sample number that has been saved successfully
	 */
	private int savedNumber;
	/**
	 * The horizontal corners number for each chessboard
	 */
	private int numCornersHor;
	/**
	 * The vertical corners nubmer for each chessboard
	 */
	private int numCornersVer;
	/**
	 * The square width of each piece in the chessboard
	 */
	private double squareWidth;
	/**
	 * The square height of each piece in the chessboard
	 */
	private double squareHeight;
	/**
	 * The original width of the captured image from camera stream
	 */
	private int imageWidth_original;
	/**
	 * The current number of recorded points 
	 */
	private int numRecord;
	/**
	 * The original height of the captured image from camera stream
	 */
	private int imageHeight_original;
	/**
	 * The intrinsic matrix of the specified camera by using camera calibration
	 */
	private Mat intrinsic;
	/**
	 * The distortion coefficients of the specified camera by using camera calibration
	 */
	private Mat distCoeffs;
	/**
	 * The rotation vector of the specified camera by using solve projection
	 */
	private Mat rvec;
	/**
	 * The translation vector of the specified camera by using solve projection
	 */
	private Mat tvec;
	/**
	 * The current state of basic camera calibration including
	 * <ul>
	 * <li>Prepare camera: 	{@code CC_PROP_STATE_PREPARE = 0}
	 * <li>Start camera: 	{@code CC_PROP_STATE_START = 1}
	 * <li>Detect corners: 	{@code CC_PROP_STATE_DETECT = 2}
	 * <li>Calibrate camera: {@code CC_PROP_STATE_CALIB = 3}
	 * <li>Undistort image: {@code CC_PROP_STATE_UNDIST = 4}
	 * <li>Close camera: 	{@code CC_PROP_STATE_CLOSE = 5}
	 * </ul>
	 */
	private int stateFlag = Const.CC_PROP_STATE_PREPARE.getValue();
	/**
	 * A flag represents that a snapshot count down thread is executed
	 */
	private boolean isWaited;
	/**
	 * A flag represents that Corner Detection is starting
	 */
//	private boolean isDetected;
	/**
	 * Whether the specified camera is calibrated or not
	 */
	private boolean isCalibrated;
	/**
	 * A flag represents that the rotation and translation vectors are solved
	 */
	private boolean isSolved;
	/**
	 * A count down period for a snapshot
	 */
	private int waitPeriod = 1500;
	private String[] keyIntrinsic = { "fx", "s", "Cx", 
									"z10", "fy", "Cy", 
									"z20", "z21", "last" };
	private String[] keyDistCoef = new String[] { "k1", "k2", "p1", "p2", 
												"k3", 
												"k4", "k5", "k6", 
												"s1", "s2", "s3", "s4", 
												"tx", "ty" };
	private String[] keyRvec = new String[] { "r0", "r1", "r2" };
	private String[] keyTvec = new String[] { "t0", "t1", "t2" };
	private String propFileName = "resources/CC.properties";
	
	// =====[Public] Property=====
	
	/**
	 * A collection of CONSTANT values for camera calibration.<p>
	 * 1. The current state enumeration of basic camera calibration including<br>
	 * <ul>
	 * <li>Prepare camera: 	{@code CC_STATE_STEP_PREPARE = 0}
	 * <li>Start camera: 	{@code CC_STATE_STEP_START = 1}
	 * <li>Detect corners: 	{@code CC_STATE_STEP_DETECT = 2}
	 * <li>Calibrate camera: {@code CC_STATE_STEP_CALIB = 3}
	 * <li>Undistort image: {@code CC_STATE_STEP_UNDIST = 4}
	 * <li>Close camera: 	{@code CC_STATE_STEP_CLOSE = 5}
	 * </ul>
	 */
	public enum Const
	{
		CC_PROP_STATE_PREPARE(0),
		CC_PROP_STATE_START(1),
		CC_PROP_STATE_DETECT(2),
		CC_PROP_STATE_CALIB(3),
		CC_PROP_STATE_UNDIST(4),
		CC_PROP_STATE_CLOSE(5);
		
		int value;
		
		private Const(int value)
		{
			this.value = value;
		}
		
		public int getValue()
		{
			return this.value;
		}
	}
	
	
	// =====[Protected] FXML Events=====
	
	/**
	 * The action triggered by pushing the button on the GUI
	 */
	@FXML
	protected void startCamera()
	{
		if (!this.cameraActive)
		{
			// Init starting
			init(Const.CC_PROP_STATE_START);
			this.cameraId = 0;
			System.loadLibrary("opencv_ffmpeg349_64");
			this.capture.set(Videoio.CV_CAP_PROP_BUFFERSIZE, 3);
			// Start the video capture
			this.capture.open(cameraId);
//			this.capture = new VideoCapture("http://192.168.1.200/GetStream.cgi");
//			this.capture.open("rtsp://192.168.1.200/v1");
			
			// Is the video stream available?
			if (this.capture.isOpened())
			{
				this.cameraActive = true;
				
				// Grab a frame every 33 ms (30 frames/sec)
				Runnable frameGrabber = new Runnable()
				{
					@Override
					public void run()
					{
						// Effectively grab and process a single frame from cam stream
						// and prepare the undistorted image if calibration has done
						grabFrame();
						
						Platform.runLater(new Runnable() {
							
							@Override
							public void run() {
								// Show the original frames
								originalFrame.setImage(CamStream);
								// Set fixed width
//								originalFrame.setFitWidth(imageWidth_original);
								// Preserve image ratio
								originalFrame.setPreserveRatio(true);
								if (!calibrateFrame.isDisable())
								{
									// Show the original frames
									calibrateFrame.setImage(procImage);
									// Preserve image ratio
									calibrateFrame.setPreserveRatio(true);
								}
							}
						});
					}
				};
				
				this.timer = Executors.newSingleThreadScheduledExecutor();
				this.timer.scheduleAtFixedRate(frameGrabber, 0, 33, TimeUnit.MILLISECONDS);
			}
			else
			{
				// Init closing (disconnecting)
				this.init(Const.CC_PROP_STATE_CLOSE);
				
				// Log the error
				System.err.println("Impossible to open the camera connection...");
				this.messageArea.setText("Impossible to open the camera connection...");
			}
		}
		else
		{
			// Init closing (disconnecting)
			this.init(Const.CC_PROP_STATE_CLOSE);
			
			// Stop the timer
			this.stopAcquisition();
		}
	}
	
	/**
	 * Take a snapshot to be used for the calibration process
	 */
	@FXML
	protected void detectCorners()
	{
		// Init detecting corners
		this.init(Const.CC_PROP_STATE_DETECT);
		
		// Total number of expected corners in the chessboard
		int numSqaures = this.numCornersHor * this.numCornersVer;
		for (int i = 0; i < numSqaures; i++)
		{
			this.obj.push_back(new MatOfPoint3f(new Point3(i / this.numCornersHor * this.squareWidth, 
														i % this.numCornersVer * this.squareHeight, 
														0.0f)));
		}
	}
	
	/**
	 * Start camera calibration once reaching the nubmer of snapshot images
	 */
	@FXML
	protected void doCalibration()
	{
		// Reach the correct number of images need for the calibration
		if (this.savedNumber >= this.boardsNumber)
		{
			// Init calibrating camera
			this.init(Const.CC_PROP_STATE_CALIB);
			
			this.calibrateCamera();
			if (this.isCalibrated)
			{
				// Update UI - Calibration Control
				this.calibrateButton.setDisable(true);
				this.undistButton.setDisable(false);
				// Update UI - Bottom button
				this.saveParamButton.setDisable(false);
			}
			else
			{
				// Update UI - Calibration Control
				this.calibrateButton.setDisable(false);
				this.undistButton.setDisable(true);
				// Update UI - Bottom button
				this.saveParamButton.setDisable(true);
			}
		}
		else
		{
			this.detectButton.setDisable(false);
			System.err.println("The number of detected images is lower than "
								+ "the specified sample number of the chessboards...");
			this.messageArea.setText("The number of detected images is lower than "
									+ "the specified sample number of the chessboards...");
		}
	}
	
	/**
	 * Prepare and show the undistored image with the calibrating parameters
	 */
	@FXML
	protected void doUndistortion()
	{
		// Init undistorting image
		this.init(Const.CC_PROP_STATE_UNDIST);
	}
	
	/**
	 * Save the camera matrix and the distortion coefficients into CC.properties of this project
	 */
	@FXML
	private void saveParameters()
	{
		if ((this.stateFlag == Const.CC_PROP_STATE_CALIB.getValue() && this.isCalibrated)
				|| (this.stateFlag == Const.CC_PROP_STATE_UNDIST.getValue() && !this.isSolved))
		{
			this.setCCProperty(this.intrinsic, this.propFileName, this.keyIntrinsic);
			this.setCCProperty(this.distCoeffs, this.propFileName, this.keyDistCoef);
		}
		else if (this.stateFlag == Const.CC_PROP_STATE_UNDIST.getValue() && this.isSolved)
		{
			this.setCCProperty(this.rvec, this.propFileName, this.keyRvec);
			this.setCCProperty(this.tvec, this.propFileName, this.keyTvec);
		}
	}
	
	/**
	 * Solve the rotation and translation vectors with the collected testing points<br>
	 * at least 4 points
	 */
	@FXML
	protected void solvePnp()
	{
		MatOfPoint3f objPointsMat = new MatOfPoint3f();
		objPointsMat.fromList(this.testObjPoints);
		MatOfPoint2f imgPointsMat = new MatOfPoint2f();
		imgPointsMat.fromList(this.testImgPoints);
		
		this.intrinsic = this.getCCProperty(new Mat(3, 3, CvType.CV_32FC1), this.propFileName, this.keyIntrinsic);
		this.distCoeffs = this.getCCProperty(new Mat(8, 1, CvType.CV_64F), this.propFileName, this.keyDistCoef);
		this.rvec = new Mat(); // rotation vectors for the certain collection of points
		this.tvec = new Mat(); // translation vector for the certain collection of points
		
		// Solve the extrinsic matrix
		Calib3d.solvePnP(objPointsMat, imgPointsMat, this.intrinsic, new MatOfDouble(this.distCoeffs), this.rvec, this.tvec);
		this.isSolved = true;
		
		Mat R = new Mat();
		Calib3d.Rodrigues(rvec, R);
		
		StringBuilder calibResult = new StringBuilder();
		calibResult.append("< Camera Position Result >\n");
		calibResult.append("Rotation Vector: \n" + this.rvec.dump() + "\n");
		calibResult.append("Rotation Matrix: \n" + R.dump() + "\n");
		calibResult.append("Translation Vector: \n" + this.tvec.dump());
		this.messageArea.setText(calibResult.toString());
		this.testArea.setDisable(false);
		this.worldX.setDisable(true);
		this.worldY.setDisable(true);
		this.worldZ.setDisable(true);
	}
	
	/**
	 * Stop and close the camera calibration process
	 */
	@FXML
	protected void closeProc()
	{
		// Stop the acquisition
		this.stopAcquisition();
		// Get a handle to the stage
		Stage stage = (Stage) this.closeButton.getScene().getWindow();
		// Close the process window
		stage.close();
	}
	
	/**
	 * Mark a test point with the real world coordinate mapping on the capturing video stream
	 */
	@FXML
	protected void markPosition()
	{
		MatOfPoint3f objPointMat = new MatOfPoint3f();
		objPointMat.fromArray(new Point3(Double.parseDouble(this.testX.getText()), 
											Double.parseDouble(this.testY.getText()), 
											Double.parseDouble(this.testZ.getText())));
		MatOfPoint2f imgPointMat = new MatOfPoint2f();
		
		this.intrinsic = this.getCCProperty(new Mat(3, 3, CvType.CV_32FC1), this.propFileName, this.keyIntrinsic);
		this.distCoeffs = this.getCCProperty(new Mat(8, 1, CvType.CV_64F), this.propFileName, this.keyDistCoef);
		this.rvec = this.getCCProperty(new Mat(3, 1, CvType.CV_64F), this.propFileName, this.keyRvec);
		this.tvec = this.getCCProperty(new Mat(3, 1, CvType.CV_64F), this.propFileName, this.keyTvec);
		
		// Projects 3D points to an image plane
		Calib3d.projectPoints(objPointMat, this.rvec, this.tvec, this.intrinsic, new MatOfDouble(this.distCoeffs), imgPointMat);
		this.testPoint = imgPointMat.toArray()[0];
	}
	
	/**
	 * Switch on realtime mode for capturing video stream with camera
	 */
	@FXML
	protected void onRealtimeMode()
	{
		// Init everything - Set UI
		this.setRCUI();
		this.selectFileButton.setDisable(true);
		
		// Init everything - Get or set data
		this.radioGroup_CM.getSelectedToggle().setUserData(1);
	}
	
	/**
	 * Switch on file mode for reading images
	 */
	@FXML
	protected void onFileMode()
	{
		// Init everything - Set UI
		this.connCamera1.setDisable(true);
		this.connCamera2.setDisable(true);
		this.selectFileButton.setDisable(false);
		
		// Init everything - Get or set data
		this.radioGroup_CM.getSelectedToggle().setUserData(2);
	}
	
	/**
	 * Check for using saved camera calibration properties <br>
	 * (intrinsic matrix, distortion coefficients) <br>
	 * and the state proceed to Undistort image<br>
	 * {@code stateFlag = CC_PROP_STATE_UNDIST}
	 * otherwise the state proceed to Start camera<br>
	 * {@code stateFlag = CC_PROP_STATE_START }
	 */
	@FXML
	protected void onCheckUse()
	{
		if (this.checkUseParam.isSelected())
		{
			// Update UI - Calibration Control
			this.detectButton.setDisable(true);
			this.calibrateButton.setDisable(true);
			this.undistButton.setDisable(false);
			// Update UI - Bottom button
			this.saveParamButton.setDisable(false);
			// Init variables - Get or set data
			this.intrinsic = getCCProperty(new Mat(3, 3, CvType.CV_32FC1), this.propFileName, this.keyIntrinsic);
			this.distCoeffs = this.getCCProperty(new Mat(8, 1, CvType.CV_64F), this.propFileName, this.keyDistCoef);
			this.isCalibrated = true;
			
			// Init undistorting image
			this.init(Const.CC_PROP_STATE_UNDIST);
		}
		else
		{
			// Init closing (disconnecting)
			this.init(Const.CC_PROP_STATE_CLOSE);
			
			// Stop the timer
			this.stopAcquisition();
		}
	}
	
	/**
	 * Get the 2D position and show it in the captured camera stream (pixels)
	 * 
	 * @param e
	 * 				A mouse-click event
	 */
	@FXML
	protected void getPosition(MouseEvent e)
	{
		if (this.stateFlag == Const.CC_PROP_STATE_UNDIST.getValue())
		{
			double scaleX = this.imageWidth_original / this.originalFrame.getBoundsInLocal().getWidth();
			double scaleY = this.imageHeight_original / this.originalFrame.getBoundsInLocal().getHeight();
			this.clickX.setText(Double.toString(e.getX() * scaleX));
			this.clickY.setText(Double.toString(e.getY() * scaleY));
			this.previewPoint = new Point(Double.parseDouble(this.clickX.getText()), 
											Double.parseDouble(this.clickY.getText()));
		}
	}
	
	/**
	 * Record the preview point position clicking on the captured camera stream (pixels)
	 */
	@FXML
	protected void recordPoint()
	{
		// Record the points into a Point list
		if (this.worldX.getText() != "" 
			&& this.worldY.getText() != "" 
			&& this.worldZ.getText() != "")
		{
			this.testImgPoints.add(this.previewPoint);
			this.testObjPoints.add(new Point3(Double.parseDouble(this.worldX.getText()), 
												Double.parseDouble(this.worldY.getText()), 
												Double.parseDouble(this.worldZ.getText())));
			this.numRecord++;
			if (this.numRecord >= 4)
			{
				this.solveButton.setDisable(false);
			}
			this.recordCount.setText(Integer.toString(this.numRecord));
			this.previewPoint = null;
			this.clickX.setText("");
			this.clickY.setText("");
			this.worldX.setText("");
			this.worldY.setText("");
			this.worldZ.setText("");
			this.testX.setText("");
			this.testY.setText("");
			this.testZ.setText("");
		}
	}
	
	/**
	 * Reset the recorded point position
	 * 
	 * @param e
	 * 				A mouse-click event
	 */
	@FXML
	protected void resetRecord(MouseEvent e)
	{
		if (e.getButton() == MouseButton.SECONDARY)
		{
			// Reset UI and variable
			this.solveButton.setDisable(true);
			this.testArea.setDisable(true);
			this.recordCount.setText(Integer.toString(0));
			this.clickX.setText("");
			this.clickY.setText("");
			this.worldX.setDisable(false);
			this.worldY.setDisable(false);
			this.worldZ.setDisable(false);
			this.worldX.setText("");
			this.worldY.setText("");
			this.worldZ.setText("");
			this.numRecord = 0;
			this.testImgPoints = new ArrayList<Point>();
			this.testObjPoints = new ArrayList<Point3>();
			this.previewPoint = null;
			this.testPoint = null;
		}
	}
	
	
	// =====[Public|protected] Method=====

	/**
	 * Initialize all the UI setup, get or set (global) variables <br>
	 * of each processing state needed in the controller <br>
	 * Use enumeration "Const" to set up the appropriate initials.
	 * 
	 * @param flag
	 * 				The current state of basic camera calibration
	 */
	protected void init(Const flag)
	{
		switch (flag)
		{
			case CC_PROP_STATE_PREPARE:
				// ==Start or restart initially==
				// Init UI - Realtime Calibration and File Calibration
				this.radioGroup_CM.selectToggle(this.radioRM);
				this.radioGroup_CM.getSelectedToggle().setUserData(1);
				this.setRCUI();
				this.selectFileButton.setDisable(true);
				// Init UI - Number of Detected Image
				this.numSaved.setText("");
				// Init UI - ImageView
				this.originalFrame.setVisible(true);
				this.originalFrame.setDisable(false);
				this.originalLabel.setVisible(false);
				this.calibrateFrame.setVisible(true);
				this.calibrateFrame.setDisable(false);
				this.procStreamLabel.setVisible(false);
				
				break;
			case CC_PROP_STATE_START:
				// ==Connect or reconnect to a camera==
				// Init UI - Realtime Calibration
				this.connCamera1.setText("Disconnect");
				// Init UI - Calibration Control
				this.detectButton.setDisable(false);
				this.checkUseParam.setDisable(false);
				// Init UI - Number of Detected Image
				this.numSaved.setText("");
				// Init UI - Record and Test Area
				this.recordArea.setDisable(true);
				this.testArea.setDisable(true);
				this.recordCount.setText("");
				// Init UI - ImageView
				this.originalFrame.setVisible(true);
				this.originalFrame.setDisable(false);
				this.originalLabel.setVisible(true);
				this.calibrateFrame.setVisible(false);
				this.calibrateFrame.setDisable(true);
				
				// Init variables - Get or set data
				this.capture = new VideoCapture();
				this.cameraActive = false;
				this.CamStream = null;
				this.procImage = null;
				this.isCalibrated = false;
				this.stateFlag = Const.CC_PROP_STATE_START.getValue();
				
				break;
			case CC_PROP_STATE_DETECT:
				// ==Detect corners in the chessboard==
				// Init UI - Calibration Control
				this.calibrateButton.setDisable(false);
				// Init UI - Number of Detected Image
				this.numSaved.setText("");
				// Init UI - ImageView
				this.calibrateFrame.setVisible(true);
				this.calibrateFrame.setDisable(false);
				this.procStreamLabel.setText("< Detected Corners >");
				this.procStreamLabel.setVisible(true);
				
				// Init variables - Get or set data
				this.savedImage = new Mat();
				this.imageCorners = new MatOfPoint2f();
				this.obj = new MatOfPoint3f();
				this.imagePoints = new ArrayList<Mat>();
				this.objectPoints = new ArrayList<Mat>();
				this.savedNumber = 0;
				this.boardsNumber = Integer.parseInt(this.numBoards.getText());
				this.numCornersHor = Integer.parseInt(this.numHorCorners.getText());
				this.numCornersVer = Integer.parseInt(this.numVertCorners.getText());
				this.squareWidth = Double.parseDouble(this.lengthHorCorners.getText());
				this.squareHeight = Double.parseDouble(this.lengthVertCorners.getText());
				this.isWaited = false;
				this.stateFlag = Const.CC_PROP_STATE_DETECT.getValue();
				
				break;
			case CC_PROP_STATE_CALIB:
				// ==Calibrate the camera==
				// Init UI - Calibration Control
				this.detectButton.setDisable(true);
				// Init UI - ImageView
				this.calibrateFrame.setDisable(true);
				
				// Init variables - Get or set data
				this.intrinsic = new Mat(3, 3, CvType.CV_32FC1);
				this.distCoeffs = new Mat(8, 1, CvType.CV_64F);
				this.stateFlag = Const.CC_PROP_STATE_CALIB.getValue();
//				this.intrinsic.put(0, 0, 1);
//				this.intrinsic.put(1, 1, 1);
				
				break;
			case CC_PROP_STATE_UNDIST:
				// ==Undistort captured images from a camera==
				// Init UI - Record and Test Area
				this.recordArea.setDisable(false);
				this.recordCount.setText(Integer.toString(0));
				// Init UI - ImageView
				this.calibrateFrame.setDisable(false);
				this.procStreamLabel.setText("< Undistorted Image >");
				
				// Init everything - Get or set data
				this.stateFlag = Const.CC_PROP_STATE_UNDIST.getValue();
				this.testImgPoints = new ArrayList<Point>();
				this.testObjPoints = new ArrayList<Point3>();
				this.numRecord = 0;
				
				break;
			case CC_PROP_STATE_CLOSE:
				// ==Disconnect from a camera==
				// Init UI - Realtime Calibration
				this.connCamera1.setText("Connect");
				// Init UI - Calibration Control
				this.calibrateButton.setDisable(true);
				this.detectButton.setDisable(true);
				this.calibrateButton.setDisable(true);
				this.undistButton.setDisable(true);
				this.checkUseParam.setDisable(true);
				// Init UI - Record and Test Area
				this.recordArea.setDisable(true);
				this.testArea.setDisable(true);
				this.recordCount.setText("");
				// Init UI - Number of Detected Image, Message Area and Bottom button
				this.numSaved.setText("");
				this.messageArea.setText("");
				this.saveParamButton.setDisable(true);
				// Init UI - ImageView
				this.originalFrame.setImage(null);
				this.originalFrame.setVisible(false);
				this.originalFrame.setDisable(false);
				this.originalLabel.setVisible(false);
				this.calibrateFrame.setImage(null);
				this.calibrateFrame.setVisible(false);
				this.calibrateFrame.setDisable(false);
				this.procStreamLabel.setVisible(false);
				
				// Init variables - Get or set data
				this.cameraActive = false;	// The camera is not active at this point
				this.stateFlag = Const.CC_PROP_STATE_CLOSE.getValue();
				
				break;
				
			default:
				System.err.println("Failed to initialize the UI setup, get or set (global) variables...");
				this.messageArea.setText("Failed to initialize the UI setup, get or set (global) variables...");
				
				break;
		}
	}
	
	/**
	 * On application close, stop the acquisition from the camera
	 */
	protected void setClosed()
	{
		this.stopAcquisition();
	}
	
	
	// =====[Private] Function=====
	
	/**
	 * Get a frame from the opened video stream (if any)
	 * 
	 * @return the {@link Mat} to show}
	 */
	private void grabFrame()
	{
		// Init everything
		Mat frame = new Mat();
		
		// Check if the capture is open
		if (this.capture.isOpened())
		{
			try
			{
				// Read the current frame
				this.capture.read(frame);
				this.imageWidth_original = (int) frame.size().width;
				this.imageHeight_original = (int) frame.size().height;
//				Imgproc.resize(frame, frame, new Size(this.imageWidth_original * 2.5, this.imageHeight_original * 2.5));
				
				// If the frame is not empty and corner detection is starting, process it
				if (!frame.empty())
				{
					// Draw mouse clicked point if any
					if (this.testImgPoints != null && this.testImgPoints.size() > 0)
					{
						// For preview (blue circle)
						if (this.previewPoint != null)
						{
							Imgproc.circle(frame, this.previewPoint, 5, new Scalar(255, 0, 0), 2);
						}
						// For marking test point (green circle)
						if (this.testPoint != null)
						{
							Imgproc.circle(frame, this.testPoint, 5, new Scalar(0, 255, 0), 2);
						}
						// For recording confirmed (red circle)
						for (Point p : this.testImgPoints)
						{
							Imgproc.circle(frame, p, 5, new Scalar(0, 0, 255), 2);
						}
					}
					else
					{
						// The first mouse clicked point for preview
						if (this.previewPoint != null)
						{
							Imgproc.circle(frame, this.previewPoint, 5, new Scalar(255, 0, 0), 2);
						}
					}
					
					// Convert the Mat object (OpenCV) to Image (JavaFX), The Captured "Cam Stream"
					this.CamStream = Utils.mat2Image2(frame);
					
					if (this.stateFlag == Const.CC_PROP_STATE_DETECT.getValue())
					{
						if (!this.isWaited)
						{
							waitRunnable wait = new waitRunnable(this.waitPeriod);
							wait.start();
							
							// Find the chessboard pattern
							if (this.findAndDrawPoints(frame))
							{
								// Convert the Mat object (OpenCV) to Image (JavaFX), The Captured "Detected Corners Image"
								this.procImage = Utils.mat2Image2(frame);
								this.takeSnapshot();
							}
						}
					}
					
					if (this.isCalibrated && this.stateFlag == Const.CC_PROP_STATE_UNDIST.getValue())
					{
						// Prepare the undistorted image
						Mat undistored = new Mat();
						Imgproc.undistort(frame, undistored, this.intrinsic, this.distCoeffs);
						// Convert the Mat object of (OpenCV) to Image (JavaFX), "Undistored Image"
						this.procImage = Utils.mat2Image2(undistored);
					}
				}
			}
			catch (Exception e)
			{
				// Log the (full) error
				System.err.println("Exception during the image elaboration: " + e);
				e.printStackTrace();
				this.messageArea.setText("Exception during the image elaboration.");
			}
		}
	}
	
	/**
	 * Take a snapshot to be used for the calibration process
	 */
	private void takeSnapshot()
	{
		// Save all the needed values
		this.imagePoints.add(this.imageCorners);
		this.imageCorners = new MatOfPoint2f();
		this.objectPoints.add(this.obj);
		this.savedNumber++;
		
		// Update the sample number that has been saved successfully
		this.numSaved.setText(Integer.toString(this.savedNumber));
	}
	
	/**
	 * Find and draws the points needed for the calibration on the chessboard
	 * 
	 * @param frame
	 *            the current frame
	 * @return return the corners of the chessboard are found or not
	 */
	private boolean findAndDrawPoints(Mat frame)
	{
		// Init everything
		Mat grayImage = new Mat();
		boolean found = false;
		
		// Convert the frame in gray scale
		Imgproc.cvtColor(frame, grayImage, Imgproc.COLOR_BGR2GRAY);
		// The size of the chessboard
		Size boardSize = new Size(this.numCornersVer, this.numCornersHor);
		// Look for the inner chessboard corners
		found = Calib3d.findChessboardCorners(grayImage, boardSize, this.imageCorners,
				Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK);
		// All the required corners have been found...
		if (found)
		{
			// Optimization
			TermCriteria term = new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.1);
			Imgproc.cornerSubPix(grayImage, this.imageCorners, new Size(11, 11), new Size(-1, -1), term);
			// Save the current frame for further elaborations
			grayImage.copyTo(this.savedImage);
			// Show the chessboard inner corners on screen
			Calib3d.drawChessboardCorners(frame, boardSize, this.imageCorners, found);
		}
		
		return found;
	}
	
	/**
	 * The effective camera calibration, to be performed once in the program execution
	 */
	private void calibrateCamera()
	{
		// init needed variables according to OpenCV docs
		List<Mat> rvecs = new ArrayList<Mat>(); // rotation vectors for all chessboard samples
		List<Mat> tvecs = new ArrayList<Mat>(); // translation vectors for all chessboard samples
		// Start calibration!
		Calib3d.calibrateCamera(this.objectPoints, this.imagePoints, this.savedImage.size(), 
								this.intrinsic, this.distCoeffs, rvecs, tvecs, 
								Calib3d.CALIB_FIX_PRINCIPAL_POINT | Calib3d.CALIB_FIX_K4 | Calib3d.CALIB_FIX_K5);
		this.isCalibrated = true;
		
		int rows = this.intrinsic.rows();
		int cols = this.intrinsic.cols();
		int channels = this.intrinsic.channels();
		String appendIntrStr1 = "";
		String appendIntrStr2 = "";
		String appendIntrStr3 = "";
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				for (int k = 0; k < channels; k++)
				{
					switch (i) {
					case 0:
						appendIntrStr1 += this.intrinsic.get(i, j)[k];
						if (((j + 1) * (k + 1)) != (cols * channels))
						{
							appendIntrStr1 += ", ";
						}
						break;
					case 1:
						appendIntrStr2 += this.intrinsic.get(i, j)[k];
						if (((j + 1) * (k + 1)) != (cols * channels))
						{
							appendIntrStr2 += ", ";
						}
						break;
					case 2:
						appendIntrStr3 += this.intrinsic.get(i, j)[k];
						if (((j + 1) * (k + 1)) != (cols * channels))
						{
							appendIntrStr3 += ", ";
						}
						break;
					default:
						break;
					}
				}
			}
		}
		
		rows = this.distCoeffs.rows();
		cols = this.distCoeffs.cols();
		channels = this.distCoeffs.channels();
		String appendDistStr = "";
		int count = 0;
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				for (int k = 0; k < channels; k++)
				{
					appendDistStr += "[" + keyDistCoef[count] + "] " + this.distCoeffs.get(i, j)[k];
					if (((i + 1) * (j + 1) * (k + 1)) != (rows * cols * channels))
					{
						appendDistStr += ", ";
						count++;
					}
				}
			}
		}
		
		// Show the results of intrinsic matrix, extrinsic matrix 
		// and distortion coefficients of the specified camera
		StringBuilder calibResult = new StringBuilder();
		calibResult.append("< Camera Calibration Result >\n");
		calibResult.append("- Calibration Date: " + LocalDateTime.now() + "\n");
		calibResult.append("- Number of Detected Images: " + this.savedNumber + "\n");
		calibResult.append("- Corners Width: " + this.numCornersHor + "\n");
		calibResult.append("- Corners Height: " + this.numCornersVer + "\n");
		calibResult.append("- Square Size of CB: [" + this.squareWidth + ", " + this.squareHeight + "] (mm)\n");
		calibResult.append("- Resolution: [" + this.imageWidth_original + ", "
											+ this.imageHeight_original + "]\n");
		calibResult.append("- Focal Lenght fx: " + this.intrinsic.get(0, 0)[0] + "\n");
		calibResult.append("- Focal Lenght fy: " + this.intrinsic.get(1, 1)[0] + "\n");
		calibResult.append("- Principal Point Cx: " + this.intrinsic.get(0, 2)[0] + "\n");
		calibResult.append("- Principal Point Cy: " + this.intrinsic.get(1, 2)[0] + "\n");
		calibResult.append("- Skew Coefficient s: " + this.intrinsic.get(0, 1)[0] + "\n");
		calibResult.append("- Distortion: " + appendDistStr + "\n");
		calibResult.append("- Camera Matrix [0]"+ appendIntrStr1 + "\n");
		calibResult.append("- Camera Matrix [1]"+ appendIntrStr2 + "\n");
		calibResult.append("- Camera Matrix [2]"+ appendIntrStr3 + "\n");
		
		this.messageArea.setText(calibResult.toString());
	}
	
	/**
	 * Stop the acquisition from the camera and release all the resources
	 */
	private void stopAcquisition()
	{
		if (this.timer != null && !this.timer.isShutdown())
		{
			try
			{
				// Stop the timer
				this.timer.shutdown();
				this.timer.awaitTermination(33, TimeUnit.MILLISECONDS);
				this.timer = null;
			}
			catch (Exception e)
			{
				// Log any exception
				System.err.println("Exception in stopping the frame capture, trying to release the camera now... " + e);
				this.messageArea.setText("Exception in stopping the frame capture, trying to release the camera now...");
			}
		}
		
		if (this.capture.isOpened())
		{
			// Release the camera
			this.capture.release();
		}
	}
	
	/**
	 * Get a list of all available camera devices
	 * 
	 * @return available camera counts
	 */
	private int enumerateCameras()
	{
		VideoCapture camera = new VideoCapture();
		int device_counts = 0;
		while (true)
		{
			if (!camera.open(device_counts))
			{
				break;
			}
			
			device_counts++;
		}
		
		return device_counts;
	}
	
	/**
	 * Set up the UI setting of Realtime Calibration block <br>
	 * if it is switch into Realtime Mode.
	 */
	private void setRCUI()
	{
		int camCount = this.enumerateCameras();
		if (camCount == 0)
		{
			this.connCamera1.setDisable(true);
			this.connCamera2.setDisable(true);
		}
		else if (camCount == 1)
		{
			this.connCamera1.setDisable(false);
			this.connCamera2.setDisable(true);
		}
		else
		{
			this.connCamera1.setDisable(false);
			this.connCamera2.setDisable(false);
		}
	}
	
	/**
	 * Set the properties value about the camera calibration including<br>
	 * camera matrix (intrinsic matrix), distortion coefficients, <br>
	 * rotation vector and translation vector
	 * 
	 * @param src
	 * 				Original data source of the camera calibration properties
	 * @param propFile
	 * 				The property file name and the file path related to the workspace
	 * @param key
	 * 				Key list for each property values
	 */
	private void setCCProperty(Mat src, String propFile, String[] key)
	{
		File f = new File(propFile);
		Properties prop = new Properties();
		
		if (f.isFile() && f.canRead())
		{
			// Try to open the stream and get the properties value
			try (InputStream input = new FileInputStream(propFile))
			{
				try
				{
					prop.load(input);
				}
				finally
				{
					input.close();
				}
			}
			catch (Exception e)
			{
				e.printStackTrace();
				System.err.println("Failed to open the property file, CC.properties...");
				this.messageArea.setText("Failed to open the property file, CC.properties...");
			}
		}
		
		// Try to open the stream and set the properties value
		try (OutputStream output = new FileOutputStream(propFile))
		{
			try
			{
				int rows = src.rows();
				int cols = src.cols();
				int channels = src.channels();
				int count = 0;
				for (int i = 0; i < rows; i++)
				{
					for (int j = 0; j < cols; j++)
					{
						for (int k = 0; k < channels; k++)
						{
							prop.setProperty(key[count], Double.toString(src.get(i, j)[k]));
							if (((i + 1) * (j + 1) * (k + 1)) != (rows * cols * channels))
							{
								count++;
							}
						}
					}
				}
				
				// Save properties to project root folder
				prop.store(output, "Property values for camera calibration.");
				System.out.println(prop);
			}
			finally
			{
				output.close();
			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
			System.err.println("Failed to save the result into the property file, CC.properties...");
			this.messageArea.setText("Failed to save the result into the property file, CC.properties...");
		}
	}
	
	/**
	 * Get the properties value about the camera calibration including<br>
	 * camera matrix (intrinsic matrix), distortion coefficients, <br>
	 * rotation vector and translation vector
	 * 
	 * @param newMat
	 * 				A new matrix with given rows, cols, and type
	 * @param propFile
	 * 				The property file name and the file path related to the workspace
	 * @param key
	 * 				Key list for each property values
	 * @return a Mat based on the size of newMat of camera calibration properties<br>
	 * 			return null if there are some exceptions such as <br>
	 * 			properties file not found, a newMat without given size, etc.
	 */
	private Mat getCCProperty(Mat newMat, String propFile, String[] key)
	{
		File f = new File(propFile);
		Properties prop = new Properties();
		
		if (f.isFile() && f.canRead())
		{
			// Try to open the stream and get the properties value
			try (InputStream input = new FileInputStream(propFile))
			{
				try
				{
					// Load a properties file
					prop.load(input);
					
					int rows = newMat.rows();
					int cols = newMat.cols();
					int channels = newMat.channels();
					int count = 0;
					for (int i = 0; i < rows; i++)
					{
						for (int j = 0; j < cols; j++)
						{
							for (int k = 0; k < channels; k++)
							{
								String propValue = prop.getProperty(key[count]);
								if (propValue != null)
								{
									newMat.put(i, j, Double.parseDouble(propValue));
									System.out.println(key[count] + ": " + newMat.get(i, j)[k]);
									if (((i + 1) * (j + 1) * (k + 1)) != (rows * cols * channels))
									{
										count++;
									}
								}
							}
						}
					}
					
					return newMat;
				}
				finally
				{
					input.close();
				}
			}
			catch (Exception e)
			{
				e.printStackTrace();
				System.err.println("Failed to open the property file, CC.properties or get the property...");
				this.messageArea.setText("Failed to open the property file, CC.properties or get the property...");
				return null;
			}
		}
		
		return null;
	}
	

	// =====[Public] Class=====
	
	/**
	 * Count down a specified time period when takeing a snapshot
	 */
	public class waitRunnable implements Runnable
	{
		// =====[Private] Variable=====
		
		private Thread worker;
		private final AtomicBoolean running = new AtomicBoolean(false);
		private int interval;
		
		
		// =====[Public] Constructor=====
		
		/**
		 * waitRunnable Constructor
		 * 
		 * @param sleepInterval
		 * 				time period to wait in miliseond(s)
		 */
		public waitRunnable(int sleepInterval)
		{
			this.interval = sleepInterval;
			isWaited = true;
		}
		
		
		// =====[Public] Method=====
		
		public void start()
		{
			this.worker = new Thread(this);
			this.worker.start();
		}
		
		public void stop()
		{
			this.running.set(false);
		}
		
		public void interrupt()
		{
			this.running.set(false);
			this.worker.interrupt();
		}
		
		boolean isRunning()
		{
			return this.running.get();
		}
		
//		boolean isStopped()
//		{
//			return !this.running.get();
//		}
		
		@Override
		public void run()
		{
			this.running.set(true);
			while (this.running.get())
			{
				try
				{
					Thread.sleep(interval);
				}
				catch (Exception e)
				{
					Thread.currentThread().interrupt();
					System.err.println("Thread was interrupted, Failed to complete operation: " + e);
					e.printStackTrace();
					messageArea.setText("Thread was interrupted, Failed to complete operation.");
				}
				
				isWaited = false;
				this.stop();
			}
		}
	}
}
