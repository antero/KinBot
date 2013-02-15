/*
KinBot
Kinect with Arduino project!

This projects needs the following libraries:

# OpenCV v2.4.3 x86
# Kinect SDK v1.6 x86
*/

#include <Windows.h>
#include <NuiApi.h>

#include <opencv2\opencv.hpp>

HANDLE m_pVideoStreamHandle;
HANDLE m_pDepthStreamHandle;
const NUI_IMAGE_FRAME * pImageFrame;
const NUI_IMAGE_FRAME * pDepthFrame;
NUI_SKELETON_FRAME skeletonFrame;
INuiFrameTexture * pImageTexture;
INuiFrameTexture * pDepthTexture;
NUI_LOCKED_RECT LockedImage;
NUI_LOCKED_RECT LockedDepth;

cv::Mat image;
cv::Mat imageDepth;

int main() {
	int device=0;
	INuiSensor* sensor;
	//cria sensor
	HRESULT hr = NuiCreateSensorByIndex(device, &sensor);
	if (hr != S_OK){
		printf("Erro ao criar sensor.\n");
		exit(-1);
	}
	
	hr = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR);
	if(hr != S_OK) {
		printf("Erro ao inicializar dispositivo.\n");
		exit(-1);
	}

	hr = NuiSkeletonTrackingEnable( 0, 0 );
    if(hr != S_OK) {
		printf("Erro ao iniciar deteccao de esqueleto.\n");
	}
		
	image = cv::Mat(480, 640, CV_8UC3);
	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &m_pVideoStreamHandle);
	
	if(hr != S_OK) {
		printf("Erro ao abrir stream RGB.\n");
		exit(-1);
	}

	imageDepth = cv::Mat(240, 320, CV_8UC3);
	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 2, 0, &m_pDepthStreamHandle);

	if(hr != S_OK) {
		printf("Erro ao abrir stream de profundidade.\n");
		exit(-1);
	}

	return 0;
}