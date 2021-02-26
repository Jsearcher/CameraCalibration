package application;
	
import org.opencv.core.Core;

import application.CC_Controller.Const;
import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.fxml.FXMLLoader;


/**
 * A classic camera calibration implemented by OpenCV with a chessboard
 *
 */
public class CameraCalibration extends Application
{
	
	@Override
	public void start(Stage primaryStage)
	{
		try {
			// Load the FXML resource
			FXMLLoader loader = new FXMLLoader(getClass().getResource("CC_FX.fxml"));
			// Store the root element so that the controllers can use it
			BorderPane rootElement = (BorderPane) loader.load();
			// Set a whitesmoke background
			rootElement.setStyle("-fx-background-color: whitesmoke;");
			// Create and style a scene
			Scene scene = new Scene(rootElement);
			scene.getStylesheets().add(getClass().getResource("application.css").toExternalForm());
			// Create the stage with the given title and the previously created
			// Scene
			primaryStage.setTitle("Camera Calibration");
			primaryStage.setScene(scene);
			// Init the controller variables
			CC_Controller controller = loader.getController();
			controller.init(Const.CC_PROP_STATE_PREPARE);
			// Set the proper behavior on closing the application
			primaryStage.setOnCloseRequest((new EventHandler<WindowEvent>() {
				
				@Override
				public void handle(WindowEvent we)
				{
					controller.setClosed();
				}
			}));
			
			// Show the GUI
			primaryStage.show();
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
	}
	
	/**
	 * For launching the application
	 * @param args
	 * 				optional params
	 */
	public static void main(String[] args)
	{
		// load the native OpenCV library
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		launch(args);
	}
}
