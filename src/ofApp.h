#pragma once

#include "ofMain.h"
#include <rs.hpp>

#define N_CAMERAS 5
#define WIDTH 1920
#define HEIGHT 1080
#define VIEWWINDOW_HEIGHT 192

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void setupFrameworks();
		void setupCameras();
		void setupViewports();

		void drawCameras();
		void drawLabels();
		void drawMonitor(int iCameraDraw);
		void drawScene(int iCameraDraw);

		void keyPressed(int key);
		
		bool initializeT265();
		bool updateT265();

		void arBegin(int i);
		void arEnd(int i);
		void translateCamByFishEyeCenter(int i);
		void translateCamByExtrinsics(int i);
		void rotateCam(int i);
		
		// Realsense
		rs2::pipeline pipe;
		rs2::config cfg;
		rs2::pipeline_profile pipe_profile;
		rs2::stream_profile fisheye_stream;
		rs2_intrinsics intrinsics;
		rs2_extrinsics extrinsics;

		rs2_pose device_pose_in_world;

		ofTexture fisheye_texture_;

		bool is_start_{ false };
		float vol;

		//camera
		ofEasyCam cam;
		ofCamera camView;
		float camPos;
		ofVec3f axis;

		//camera pointers
		ofCamera* cameras[N_CAMERAS];
		int iMainCamera;

		//viewports
		ofRectangle viewMain;
		ofRectangle viewGrid[N_CAMERAS];

		//Light
		ofLight light;

		ofSpherePrimitive sphere;

	private:
		static const int FISH_EYE_SENSOR_USE_INDEX = 1;
};
