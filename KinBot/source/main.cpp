/*
KinBot
Kinect with Arduino project!

This projects needs the following libraries:

# OpenCV v2.4.3 x86
# Kinect SDK v1.6 x86
*/

#include "skelAngles.h"
#include "arduino_comm.h"
#include <opencv2\opencv.hpp>

#include <pthread.h>
#include <vector>

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

int angle[6] = {-1,-1,-1,-1,-1,-1};

bool valid = false;

void* streamArduino(void* arg) {
	static int lastangle[6] = {-1,-1,-1,-1,-1,-1};
	static int relativeDiff = 2;
	HRESULT hr = connectArduino();
	if (hr != S_OK){
		printf("Erro ao abrir comunicacao com arduino\n");
		exit(0);
	}

	while(true) {
		
		if (abs(lastangle[MOTOR_BASE]-angle[MOTOR_BASE])>relativeDiff){
				printf("sending command to arduino... %d: %d\n", MOTOR_BASE, angle[MOTOR_BASE]);
				if(angle[MOTOR_BASE] < 180) send2Ard(MOTOR_BASE, angle[MOTOR_BASE]);
				lastangle[MOTOR_BASE] = angle[MOTOR_BASE];
		}
		if (abs(lastangle[MOTOR_OMBRO]-angle[MOTOR_OMBRO])>relativeDiff){
				//printf("sending command to arduino... %d: %d\n", MOTOR_OMBRO, angle[MOTOR_OMBRO]);
				if(angle[MOTOR_OMBRO] < 180) send2Ard(MOTOR_OMBRO, angle[MOTOR_OMBRO]);
				lastangle[MOTOR_OMBRO] = angle[MOTOR_OMBRO];
		}
		if (abs(lastangle[MOTOR_COTOVELO]-angle[MOTOR_COTOVELO])>relativeDiff){
			//	printf("sending command to arduino... %d: %d\n", MOTOR_COTOVELO, angle[MOTOR_COTOVELO]);
				if(angle[MOTOR_COTOVELO] < 180) send2Ard(MOTOR_COTOVELO, angle[MOTOR_COTOVELO]);
				lastangle[MOTOR_COTOVELO] = angle[MOTOR_COTOVELO];
		}
		if (abs(lastangle[MOTOR_PULSO]-angle[MOTOR_PULSO])>relativeDiff){
			//	printf("sending command to arduino... %d: %d\n", MOTOR_PULSO, angle[MOTOR_PULSO]);
				if(angle[MOTOR_PULSO] < 180) send2Ard(MOTOR_PULSO, angle[MOTOR_PULSO]);
				lastangle[MOTOR_PULSO] = angle[MOTOR_PULSO];
		}
		if (abs(lastangle[MOTOR_GARRA]-angle[MOTOR_GARRA])>relativeDiff){
				printf("sending command to arduino... %d: %d\n", MOTOR_GARRA, angle[MOTOR_GARRA]);
				if(angle[MOTOR_GARRA] < 180) send2Ard(MOTOR_GARRA, angle[MOTOR_GARRA]);
				lastangle[MOTOR_GARRA] = angle[MOTOR_GARRA];
		}


		//if(abs(lastangle-angle)>5) {
		//	
		//	printf("sending command to arduino... %d: %d\n", motor, angle);
		//	
		//	if(angle < 180) send2Ard(motor, angle);
		//	
		//	lastangle = angle;
		//	
		//	//valid = false;
		//}
		
		Sleep(10);
	}


}

int main() {
	int device=0;
	INuiSensor* sensor;
#pragma region begin_streams
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
#pragma endregion

	//cria janelas
	cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Requeleto", cv::WINDOW_AUTOSIZE);

	memset(joints, 0, sizeof(FLOAT)*2*NUI_SKELETON_POSITION_COUNT);
	int count = 0;

	//inicia comunicação com robô
	if (pthread_create(&thread_c, NULL, streamArduino, NULL)) {
	    printf("Failed to create arduino communication thread.\n");
		exit(0);
	}

	//variaveis para profundidade
	//int xC=-1,yC=-1;
	//USHORT positions[9], pescoco[9];
	while (true) 
	{
		//zera imagens
		memset(image.data,0,sizeof(char)*3*640*480);
		memset(imageDepth.data,0,sizeof(char)*3*320*240);
		memset(imageSkel.data,0,sizeof(char)*3*320*240);
//		memset(positions, 0, sizeof(USHORT)*9);
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
		
		//pega orientacoes
		if (skelIndex >= 0){
			Vector4 j1 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_SPINE];//espinha dorsal
			Vector4 j2 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER];//pescoço
			Vector4 j8 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT];//ombro direito
			Vector4 j9 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT];//cotovelo direito
			Vector4 j10 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT];//pulso direito
			Vector4 j11 = skeletonFrame.SkeletonData[skelIndex].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];//mão direita
			int anguloOmbro, anguloCotovelo, anguloPulso;
		
			//profundidade
			//getJointDepth(pDepth, j2, xC, yC, pescoco);
			//getJointDepth(pDepth, j11, xC, yC, positions);
			//if (!(count%50)){
			//	int atual=0;
			//	int atual2=0;
			//	printf("NECK DEPTH = %hd %hd %hd %hd %hd %hd %hd %hd %hd\n\n", pescoco[atual2++],pescoco[atual2++],pescoco[atual2++],
			//																		pescoco[atual2++],pescoco[atual2++],pescoco[atual2++],
			//																		pescoco[atual2++],pescoco[atual2++],pescoco[atual2++]);
			//	printf("HAND DEPTH = %hd %hd %hd %hd %hd %hd %hd %hd %hd\n", positions[atual++],positions[atual++],positions[atual++],
			//																		positions[atual++],positions[atual++],positions[atual++],
			//																		positions[atual++],positions[atual++],positions[atual++]);
			//}

			//angulo do ombro
			Vector4 spineVector, armVector;
			vecsub(j1,j2,spineVector); vecsub(j9,j8,armVector);
			hr = twoVectorAngle(spineVector,armVector,MOTOR_OMBRO,anguloOmbro);
			if (hr == S_OK){
				if (!(count%50)) printf("OMBRO - %d graus :: ", anguloOmbro);
				if(anguloOmbro > 0) {
					angle[MOTOR_OMBRO] = anguloOmbro;
				}
			}
			//else printf("Nao pode pegar o angulo do ombro\n");
			//angulo do cotovelo
			hr = threeJointAngle(j8,j9,j10,MOTOR_COTOVELO,anguloCotovelo);
			if (hr == S_OK){
				if (!(count%50)) printf("COTOVELO - %d graus :: \n", anguloCotovelo);
				
				if(anguloCotovelo > 0) {
					angle[MOTOR_COTOVELO] = anguloCotovelo;
				}
			}
			//else printf("Nao pode pegar o angulo do cotovelo\n");
			//angulo do pulso
			hr = threeJointAngle(j9,j10,j11,MOTOR_PULSO,anguloPulso);
			if (hr == S_OK){
				if (!(count%50)) printf("PULSO - %d graus\n\n", anguloPulso);
				if(anguloPulso > 0) {
					angle[MOTOR_PULSO] = anguloPulso;
				}
			}
			//else printf("Nao pode pegar o angulo do pulso\n\n");			
		}
		//else{
		//	xC=-1;
		//	yC=-1;
		//}
		//if(xC>=0){ 
		//	cv::circle(imageDepth, cv::Point(xC, yC), 2, cv::Scalar(87,122,185,0));
		//}
		count++;

		//joga imagens na tela
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

	cvWaitKey();
	sensor->NuiShutdown();
	shutdownArduino();
	cv::destroyAllWindows();

	return 0;
}