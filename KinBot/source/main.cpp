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

cv::Mat image;
cv::Mat imageDepth;
cv::Mat imageSkel;

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

void grabSkel(){
	bool success = false;
	for(int i = 0; i < NUI_SKELETON_COUNT; i++){
		if (skeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_NOT_TRACKED) continue;
		success = true;
		for(int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++){
			NuiTransformSkeletonToDepthImage(skeletonFrame.SkeletonData[i].SkeletonPositions[j],
				&(joints[j][0]), &(joints[j][1]), NUI_IMAGE_RESOLUTION_320x240);
		}
		break;
	}
	if (success)
		for(int i = 0; i < NUI_SKELETON_POSITION_COUNT; i++){
			int x = (int) joints[i][0];
			int y = (int) joints[i][1];
			cv::circle(imageSkel, cv::Point(x, y), 1, cv::Scalar(0,255,0,0));
			printf("%d %d\n", x, y);
		}
}

int main() {
	int device=0;
	INuiSensor* sensor;
	//cria sensor
	HRESULT hr = NuiCreateSensorByIndex(device, &sensor);
	if (hr != S_OK){
		printf("Erro ao criar sensor.\n");
		exit(-1);
	}
	//inicia dispositivo
	hr = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR);
	if(hr != S_OK) {
		printf("Erro ao inicializar dispositivo.\n");
		exit(-1);
	}
	//define detec��o de esqueleto
	imageSkel = cv::Mat(240, 320, CV_8UC3);
	hr = sensor->NuiSkeletonTrackingEnable( 0, 0 );
    if(hr != S_OK) {
		printf("Erro ao iniciar deteccao de esqueleto.\n");
	}
		
	//abre stream RGB
	image = cv::Mat(480, 640, CV_8UC3);
	hr = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &m_pVideoStreamHandle);
	
	if(hr != S_OK) {
		printf("Erro ao abrir stream RGB.\n");
		exit(-1);
	}
	//abre stream de profundidade
	imageDepth = cv::Mat(240, 320, CV_8UC3);
	hr = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 2, 0, &m_pDepthStreamHandle);

	if(hr != S_OK) {
		printf("Erro ao abrir stream de profundidade.\n");
		exit(-1);
	}

	cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Requeleto", cv::WINDOW_AUTOSIZE);

	memset(joints, 0, sizeof(FLOAT)*2*NUI_SKELETON_POSITION_COUNT);
	//exibe streams
	while (true) 
	{
		memset(image.data,0,sizeof(char)*3*640*480);
		memset(imageDepth.data,0,sizeof(char)*3*320*240);
		memset(imageSkel.data,0,sizeof(char)*3*320*240);

		hr = sensor->NuiImageStreamGetNextFrame(m_pVideoStreamHandle, 100, &pImageFrame);
		if(hr != S_OK) {
			printf("Erro ao ler frame do stream RGB.\n");
			continue;
		}

		grabRGB();

		hr = sensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 100, &pDepthFrame);
		if(hr != S_OK) {
			printf("Erro ao ler frame do stream de profundidade.\n");
			sensor->NuiImageStreamReleaseFrame(m_pVideoStreamHandle, &pImageFrame);
			continue;
		}

		grabDepth();

		hr = sensor->NuiSkeletonGetNextFrame(100, &skeletonFrame);
		if(hr != S_OK) {
			printf("Erro ao ler frame do stream do esqueleto.\n");
			sensor->NuiImageStreamReleaseFrame(m_pVideoStreamHandle, &pImageFrame);
			sensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &pDepthFrame);
			continue;
		}

		grabSkel();

		imshow("RGB", image);
		imshow("Depth", imageDepth);
		imshow("Requeleto", imageSkel);
		
		char c = cvWaitKey(10);
		if((char) c == 27 ) {
			break;
		}

		sensor->NuiImageStreamReleaseFrame(m_pVideoStreamHandle, &pImageFrame);
		sensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &pDepthFrame);
		
	}
	sensor->NuiShutdown();
	cv::destroyAllWindows();

	return 0;
}