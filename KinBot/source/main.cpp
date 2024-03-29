/*
KinBot
Kinect with Arduino project!

This projects requires the following libraries:

# OpenCV v2.4.3 x86
# Kinect SDK v1.6 x86
# pthreads

By GRVM team!
*/

#include "skelAngles.h"
#include "arduino_comm.h"
#include <opencv2\opencv.hpp>

#include <pthread.h>

HANDLE m_pVideoStreamHandle;
HANDLE m_pDepthStreamHandle;
NUI_IMAGE_FRAME pImageFrame;
NUI_IMAGE_FRAME pDepthFrame;
NUI_SKELETON_FRAME skeletonFrame;
INuiFrameTexture * pImageTexture;
INuiFrameTexture * pDepthTexture;
NUI_LOCKED_RECT LockedImage;
NUI_LOCKED_RECT LockedDepth;
BYTE * pRGB;
USHORT * pDepth;
FLOAT joints[NUI_SKELETON_POSITION_COUNT][2];
NUI_SKELETON_BONE_ORIENTATION orientations[NUI_SKELETON_POSITION_COUNT];
int skelIndex = -1;

pthread_t thread_c;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

cv::Mat image;
cv::Mat imageDepth;
cv::Mat imageSkel;

int angle[6] = {-1,-1,-1,-1,-1,-1};

void grabRGB(){
	pImageTexture = pImageFrame.pFrameTexture;
	pImageTexture->LockRect( 0, &LockedImage, NULL, 0);
	pRGB = (BYTE*) LockedImage.pBits;

	for(int i = 0; i < 640 * 480; i++)
	{
		image.data[i*3 + 0] = pRGB[i*4 + 0];
		image.data[i*3 + 1] = pRGB[i*4 + 1];
		image.data[i*3 + 2] = pRGB[i*4 + 2];
	}
}

void grabDepth(){
	pDepthTexture = pDepthFrame.pFrameTexture;
	pDepthTexture->LockRect( 0, &LockedDepth, NULL, 0);
	pDepth = (USHORT*) LockedDepth.pBits;

	for(int i = 0; i < 320 * 240; i++)
	{
		USHORT sDepth = pDepth[i] >> 3;
		char depth = 0;
		if(sDepth != 0) {
			depth = 255 - ((sDepth - 800) / 3200.0f) * 255;
		}
		
		if ((pDepth[i] & 7) == 0)
		{
			imageDepth.data[i*3+0] = depth;
			imageDepth.data[i*3+1] = depth;
			imageDepth.data[i*3+2] = depth;
		} else 
		{
			imageDepth.data[i*3+0] = pDepth[i] & 1 ? depth : 0;
			imageDepth.data[i*3+1] = pDepth[i] & 2 ? depth : 0;
			imageDepth.data[i*3+2] = pDepth[i] & 4 ? depth : 0;
		}
	}
}

void drawSkelLines(){
	cv::Scalar color = cv::Scalar(39,127,255,0);
	cv::line(imageSkel, cv::Point((int) joints[0][0], (int) joints[0][1]), cv::Point((int) joints[1][0], (int) joints[1][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[0][0], (int) joints[0][1]), cv::Point((int) joints[12][0], (int) joints[12][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[0][0], (int) joints[0][1]), cv::Point((int) joints[16][0], (int) joints[16][1]), color, 2);
	//cv::line(imageSkel, cv::Point((int) joints[1][0], (int) joints[1][1]), cv::Point((int) joints[2][0], (int) joints[2][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[1][0], (int) joints[1][1]), cv::Point((int) joints[4][0], (int) joints[4][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[1][0], (int) joints[1][1]), cv::Point((int) joints[8][0], (int) joints[8][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[2][0], (int) joints[2][1]), cv::Point((int) joints[3][0], (int) joints[3][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[2][0], (int) joints[2][1]), cv::Point((int) joints[4][0], (int) joints[4][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[2][0], (int) joints[2][1]), cv::Point((int) joints[8][0], (int) joints[8][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[4][0], (int) joints[4][1]), cv::Point((int) joints[5][0], (int) joints[5][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[5][0], (int) joints[5][1]), cv::Point((int) joints[6][0], (int) joints[6][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[6][0], (int) joints[6][1]), cv::Point((int) joints[7][0], (int) joints[7][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[8][0], (int) joints[8][1]), cv::Point((int) joints[9][0], (int) joints[9][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[9][0], (int) joints[9][1]), cv::Point((int) joints[10][0], (int) joints[10][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[10][0], (int) joints[10][1]), cv::Point((int) joints[11][0], (int) joints[11][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[12][0], (int) joints[12][1]), cv::Point((int) joints[13][0], (int) joints[13][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[13][0], (int) joints[13][1]), cv::Point((int) joints[14][0], (int) joints[14][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[14][0], (int) joints[14][1]), cv::Point((int) joints[15][0], (int) joints[15][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[16][0], (int) joints[16][1]), cv::Point((int) joints[17][0], (int) joints[17][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[17][0], (int) joints[17][1]), cv::Point((int) joints[18][0], (int) joints[18][1]), color, 2);
	cv::line(imageSkel, cv::Point((int) joints[18][0], (int) joints[18][1]), cv::Point((int) joints[19][0], (int) joints[19][1]), color, 2);
}

void grabSkel(){
	bool success = false;
	for(int i = 0; i < NUI_SKELETON_COUNT; i++){
		if (skeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_NOT_TRACKED) continue;
		success = true;
		skelIndex = i;
		for(int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++){
			NuiTransformSkeletonToDepthImage(skeletonFrame.SkeletonData[i].SkeletonPositions[j],
				&(joints[j][0]), &(joints[j][1]), NUI_IMAGE_RESOLUTION_320x240);
		}
		break;
	}
	if (success){

		for(int i = 0; i < 320 * 240; i++)
		{
			uchar b = imageDepth.data[i*3+0];
			uchar g = imageDepth.data[i*3+1];
			uchar r = imageDepth.data[i*3+2];
			if (!(b==g && b==r)){
				imageSkel.data[i*3+0] = 232;
				imageSkel.data[i*3+1] = 162;
				imageSkel.data[i*3+2] = 0;
			}
		}
		
		for(int i = 0; i < NUI_SKELETON_POSITION_COUNT; i++){
			int x = (int) joints[i][0];
			int y = (int) joints[i][1];
			cv::circle(imageSkel, cv::Point(x, y), 3, cv::Scalar(0,0,255,0), -1);
		}

		drawSkelLines();
	}
}

void* streamArduino(void* arg) {
	static int lastangle[6] = {-1,-1,-1,-1,-1,-1};
	static int relativeDiff = 2;
	HRESULT hr = connectArduino();
	if (hr != S_OK){
		printf("Erro ao abrir comunicacao com arduino\n");
		exit(0);
	}

	while(true) {
		
		if (abs(lastangle[SERVO_BASE]-angle[SERVO_BASE])>relativeDiff){
				//printf("sending command to arduino... %d: %d\n", SERVO_BASE, angle[SERVO_BASE]);
				if(angle[SERVO_BASE] < 180) send2Ard(SERVO_BASE, angle[SERVO_BASE]);
				lastangle[SERVO_BASE] = angle[SERVO_BASE];
		}
		if (abs(lastangle[SERVO_SHOULDER]-angle[SERVO_SHOULDER])>relativeDiff){
				//printf("sending command to arduino... %d: %d\n", SERVO_SHOULDER, angle[SERVO_SHOULDER]);
				if(angle[SERVO_SHOULDER] < 180) send2Ard(SERVO_SHOULDER, angle[SERVO_SHOULDER]);
				lastangle[SERVO_SHOULDER] = angle[SERVO_SHOULDER];
		}
		if (abs(lastangle[SERVO_ELBOW]-angle[SERVO_ELBOW])>relativeDiff){
				//printf("sending command to arduino... %d: %d\n", SERVO_ELBOW, angle[SERVO_ELBOW]);
				if(angle[SERVO_ELBOW] < 180) send2Ard(SERVO_ELBOW, angle[SERVO_ELBOW]);
				lastangle[SERVO_ELBOW] = angle[SERVO_ELBOW];
		}
		if (abs(lastangle[SERVO_WRIST]-angle[SERVO_WRIST])>relativeDiff){
				//printf("sending command to arduino... %d: %d\n", SERVO_WRIST, angle[SERVO_WRIST]);
				if(angle[SERVO_WRIST] < 180) send2Ard(SERVO_WRIST, angle[SERVO_WRIST]);
				lastangle[SERVO_WRIST] = angle[SERVO_WRIST];
		}
		if (abs(lastangle[SERVO_HAND]-angle[SERVO_HAND])>relativeDiff){
				//printf("sending command to arduino... %d: %d\n", SERVO_HAND, angle[SERVO_HAND]);
				if(angle[SERVO_HAND] < 180) send2Ard(SERVO_HAND, angle[SERVO_HAND]);
				lastangle[SERVO_HAND] = angle[SERVO_HAND];
		}
		
		Sleep(10);
	}


}

int main() {
	int device=0;
	INuiSensor* sensor;

	//depth variables
	USHORT arr_rightHand_depth[JOINT_NEIGHBORHOOD], arr_leftHand_depth[JOINT_NEIGHBORHOOD], arr_neck_depth[JOINT_NEIGHBORHOOD];
	int depth_diff = 300;

#pragma region begin_streams
	//create sensor
	HRESULT hr = NuiCreateSensorByIndex(device, &sensor);
	if (hr != S_OK){
		printf("Erro ao criar sensor.\n");
		exit(-1);
	}
	//start device
	hr = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR);
	if(hr != S_OK) {
		printf("Erro ao inicializar dispositivo.\n");
		exit(-1);
	}
	//define skeleton detection
	imageSkel = cv::Mat(240, 320, CV_8UC3);
	hr = sensor->NuiSkeletonTrackingEnable( 0, 0 );
    if(hr != S_OK) {
		printf("Erro ao iniciar deteccao de esqueleto.\n");
	}
		
	//open RGB stream
	image = cv::Mat(480, 640, CV_8UC3);
	hr = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &m_pVideoStreamHandle);
	if(hr != S_OK) {
		printf("Erro ao abrir stream RGB.\n");
		exit(-1);
	}
	//open depth stream
	imageDepth = cv::Mat(240, 320, CV_8UC3);
	hr = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 2, 0, &m_pDepthStreamHandle);
	if(hr != S_OK) {
		printf("Erro ao abrir stream de profundidade.\n");
		exit(-1);
	}
#pragma endregion

	//initialize windows
	cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Skeleton", cv::WINDOW_AUTOSIZE);

	memset(joints, 0, sizeof(FLOAT)*2*NUI_SKELETON_POSITION_COUNT);
	int count = 0;

	//start arduino communication
	if (pthread_create(&thread_c, NULL, streamArduino, NULL)) {
	    printf("Failed to create arduino communication thread.\n");
		exit(0);
	}

	while (true) 
	{
		//reset images
		memset(image.data,0,sizeof(char)*3*640*480);
		memset(imageDepth.data,0,sizeof(char)*3*320*240);
		memset(imageSkel.data,0,sizeof(char)*3*320*240);
		memset(arr_rightHand_depth, 0, sizeof(USHORT)*JOINT_NEIGHBORHOOD);
		memset(arr_leftHand_depth, 0, sizeof(USHORT)*JOINT_NEIGHBORHOOD);
		memset(arr_neck_depth, 0, sizeof(USHORT)*JOINT_NEIGHBORHOOD);
		
		//color image
		hr = sensor->NuiImageStreamGetNextFrame(m_pVideoStreamHandle, 100, &pImageFrame);
		if(hr != S_OK) {
			printf("Erro ao ler frame do stream RGB.\n");
			continue;
		}
		grabRGB();
		//depth image
		hr = sensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 100, &pDepthFrame);
		if(hr != S_OK) {
			printf("Erro ao ler frame do stream de profundidade.\n");
			sensor->NuiImageStreamReleaseFrame(m_pVideoStreamHandle, &pImageFrame);
			continue;
		}
		grabDepth();
		//skeleton image
		hr = sensor->NuiSkeletonGetNextFrame(100, &skeletonFrame);
		if(hr != S_OK) {
			printf("Erro ao ler frame do stream do esqueleto.\n");
			sensor->NuiImageStreamReleaseFrame(m_pVideoStreamHandle, &pImageFrame);
			sensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &pDepthFrame);
			continue;
		}
		grabSkel();
		
		//grab orientations
		if (skelIndex >= 0){
			Vector4 j1	= skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_SPINE];
			Vector4 j2	= skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER];
			Vector4 j7	= skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT];
			Vector4 j8	= skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT];
			Vector4 j9	= skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT];
			Vector4 j10 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT];
			Vector4 j11 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];
		
			int angle_shoulder=-1, angle_elbow=-1, angle_wrist=-1;
		
			//grab depth
			getJointDepth(pDepth, j11, arr_rightHand_depth);
			getJointDepth(pDepth, j7 , arr_leftHand_depth);
			getJointDepth(pDepth, j2 , arr_neck_depth);

			USHORT right_hand_depth = getMedian(arr_rightHand_depth);
			USHORT left_hand_depth = getMedian(arr_leftHand_depth);
			USHORT neck_depth = getMedian(arr_neck_depth);

			//move base around
			if (abs(neck_depth-left_hand_depth) > depth_diff){
				if (neck_depth > left_hand_depth){
					if (angle[SERVO_BASE] + 1 <= 180){
						angle[SERVO_BASE] += 1;
					}
				}
				else{
					if (angle[SERVO_BASE] - 1 >= 0){
						angle[SERVO_BASE] -= 1;
					}
				}
			}

			//right shoulder angle
			Vector4 spineVector, armVector;
			vecsub(j1,j2,spineVector); vecsub(j9,j8,armVector);
			hr = twoVectorAngle(spineVector,armVector,SERVO_SHOULDER,angle_shoulder);
			if (hr == S_OK){
				if (!(count%50)) printf("SHOULDER - %d degrees :: ", angle_shoulder);
				if(angle_shoulder > 0) {
					angle[SERVO_SHOULDER] = angle_shoulder;
				}
			}
			//else printf("Cannot grab right shoulder angle\n");

			//right elbow angle
			hr = threeJointAngle(j8,j9,j10,SERVO_ELBOW,angle_elbow);
			if (hr == S_OK){
				if (!(count%50)) printf("ELBOW - %d degrees :: \n", angle_elbow);
				
				if(angle_elbow > 0) {
					angle[SERVO_ELBOW] = angle_elbow;
				}
			}
			//else printf("Cannot grab right elbow angle\n");
			
			//right wrist angle
			hr = threeJointAngle(j9,j10,j11,SERVO_WRIST,angle_wrist);
			if (hr == S_OK){
				if (!(count%50)) printf("WRIST - %d degrees :: ", angle_wrist);
				if(angle_wrist > 0) {
					angle[SERVO_WRIST] = angle_wrist;
				}
			}
			//else printf("Cannot grab right wrist angle\n\n");

			//open and close claw
			if (abs(neck_depth-right_hand_depth) > depth_diff){
				if (neck_depth > right_hand_depth){
					if (angle[SERVO_HAND] + 1 <= 180){
						angle[SERVO_HAND] += 1;
					}
				}
				else{
					if (angle[SERVO_HAND] - 1 >= 0){
						angle[SERVO_HAND] -= 1;
					}
				}
				if (!(count%50)) printf("CLAW - %u degrees\n\n", angle[SERVO_HAND]);
			}
		}

		count++;

		//render images on screen
		imshow("RGB", image);
		imshow("Depth", imageDepth);
		imshow("Skeleton", imageSkel);
		char c = cvWaitKey(10);
		if((char) c == 27 ) {
			break;
		}

		sensor->NuiImageStreamReleaseFrame(m_pVideoStreamHandle, &pImageFrame);
		sensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &pDepthFrame);
		
	}

	cvWaitKey();
	sensor->NuiShutdown();
	shutdownArduino();
	cv::destroyAllWindows();

	return 0;
}