/*
KinBot
Kinect with Arduino project!

This projects needs the following libraries:

# OpenCV v2.4.3 x86
# Kinect SDK v1.6 x86
*/

#include "skelAngles.h"

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
NUI_SKELETON_BONE_ORIENTATION orientations[NUI_SKELETON_POSITION_COUNT];
int skelIndex = -1;

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

void drawSkelLines(){
	cv::line(imageSkel, cv::Point((int) joints[0][0], (int) joints[0][1]), cv::Point((int) joints[1][0], (int) joints[1][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[0][0], (int) joints[0][1]), cv::Point((int) joints[12][0], (int) joints[12][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[0][0], (int) joints[0][1]), cv::Point((int) joints[16][0], (int) joints[16][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[1][0], (int) joints[1][1]), cv::Point((int) joints[2][0], (int) joints[2][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[2][0], (int) joints[2][1]), cv::Point((int) joints[3][0], (int) joints[3][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[2][0], (int) joints[2][1]), cv::Point((int) joints[4][0], (int) joints[4][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[2][0], (int) joints[2][1]), cv::Point((int) joints[8][0], (int) joints[8][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[4][0], (int) joints[4][1]), cv::Point((int) joints[5][0], (int) joints[5][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[5][0], (int) joints[5][1]), cv::Point((int) joints[6][0], (int) joints[6][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[6][0], (int) joints[6][1]), cv::Point((int) joints[7][0], (int) joints[7][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[8][0], (int) joints[8][1]), cv::Point((int) joints[9][0], (int) joints[9][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[9][0], (int) joints[9][1]), cv::Point((int) joints[10][0], (int) joints[10][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[10][0], (int) joints[10][1]), cv::Point((int) joints[11][0], (int) joints[11][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[12][0], (int) joints[12][1]), cv::Point((int) joints[13][0], (int) joints[13][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[13][0], (int) joints[13][1]), cv::Point((int) joints[14][0], (int) joints[14][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[14][0], (int) joints[14][1]), cv::Point((int) joints[15][0], (int) joints[15][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[16][0], (int) joints[16][1]), cv::Point((int) joints[17][0], (int) joints[17][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[17][0], (int) joints[17][1]), cv::Point((int) joints[18][0], (int) joints[18][1]), cv::Scalar(0,0,255,0));
	cv::line(imageSkel, cv::Point((int) joints[18][0], (int) joints[18][1]), cv::Point((int) joints[19][0], (int) joints[19][1]), cv::Scalar(0,0,255,0));
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
		
		for(int i = 0; i < NUI_SKELETON_POSITION_COUNT; i++){
			int x = (int) joints[i][0];
			int y = (int) joints[i][1];
			cv::circle(imageSkel, cv::Point(x, y), 1, cv::Scalar(0,255,0,0));
		}

		drawSkelLines();
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
	//define detecção de esqueleto
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
	int count = 0;
	while (true) 
	{
		//zera imagens
		memset(image.data,0,sizeof(char)*3*640*480);
		memset(imageDepth.data,0,sizeof(char)*3*320*240);
		memset(imageSkel.data,0,sizeof(char)*3*320*240);
		//imagem de cor
		hr = sensor->NuiImageStreamGetNextFrame(m_pVideoStreamHandle, 100, &pImageFrame);
		if(hr != S_OK) {
			printf("Erro ao ler frame do stream RGB.\n");
			continue;
		}
		grabRGB();
		//imagem de profundidade
		hr = sensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 100, &pDepthFrame);
		if(hr != S_OK) {
			printf("Erro ao ler frame do stream de profundidade.\n");
			sensor->NuiImageStreamReleaseFrame(m_pVideoStreamHandle, &pImageFrame);
			continue;
		}
		grabDepth();
		//imagem do esqueleto
		hr = sensor->NuiSkeletonGetNextFrame(100, &skeletonFrame);
		if(hr != S_OK) {
			printf("Erro ao ler frame do stream do esqueleto.\n");
			sensor->NuiImageStreamReleaseFrame(m_pVideoStreamHandle, &pImageFrame);
			sensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &pDepthFrame);
			continue;
		}
		grabSkel();
		//joga imagens na tela
		imshow("RGB", image);
		imshow("Depth", imageDepth);
		imshow("Requeleto", imageSkel);
		
		//pega orientacoes
		if (skelIndex >= 0){
			Vector4 j4 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[4];
			Vector4 j8 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[8];
			Vector4 j9 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[9];
			Vector4 j10 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[10];
			Vector4 j11 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[11];
			int anguloOmbro = threeJointAngle(j4,j8,j9);
			int anguloCotovelo = threeJointAngle(j8,j9,j10);
			int anguloPulso = threeJointAngle(j9,j10,j11);
			printf("O J4 - J8 - J9 = %d graus\n", anguloOmbro);
			printf("C J8 - J9 - J10 = %d graus\n", anguloCotovelo);
			printf("P J9 - J10 - J11 = %d graus\n\n", anguloPulso);
		}

		char c = cvWaitKey(10);
		if((char) c == 27 ) {
			break;
		}

		sensor->NuiImageStreamReleaseFrame(m_pVideoStreamHandle, &pImageFrame);
		sensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &pDepthFrame);
		
	}
	cvWaitKey();
	sensor->NuiShutdown();
	cv::destroyAllWindows();

	return 0;
}