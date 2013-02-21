#ifndef SKELETON_ANGLES

#define SKELETON_ANGLES

#define PI 3.14159265

#include <Windows.h>
#include <NuiApi.h>
#include <math.h>

#define MOTOR_BASE		1
#define MOTOR_OMBRO		2
#define MOTOR_COTOVELO  3
#define MOTOR_PULSO		4
#define MOTOR_GARRA		5

void vecsub(Vector4 vec1, Vector4 vec2, Vector4 &res){
	res.x = vec1.x - vec2.x;
	res.y = vec1.y - vec2.y;
	res.z = vec1.z - vec2.z;
}

float dotproduct(Vector4 vec1, Vector4 vec2){
	return (vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z);
}

float module(Vector4 vec){
	return sqrt( dotproduct(vec, vec) );
}

//retorn angulo entre 3 juntas
HRESULT threeJointAngle(Vector4 j1, Vector4 j2, Vector4 j3, int motor, int &result){
	Vector4 v1, v2;
	vecsub(j1, j2, v1);
	vecsub(j3, j2, v2);
	float cosTeta = dotproduct(v1, v2) / (module(v1) * module(v2)) ;
	float degree = acos(cosTeta) * 180.0 / PI;
	
	FLOAT xP=0, yP=0, xO=0, yO=0;
	switch (motor){
	case MOTOR_BASE:
		break;
	case MOTOR_OMBRO:
		degree = degree - 90;
		break;
	case MOTOR_COTOVELO:
		NuiTransformSkeletonToDepthImage(j1, &(xO), &(yO), NUI_IMAGE_RESOLUTION_320x240);
		NuiTransformSkeletonToDepthImage(j3, &(xP), &(yP), NUI_IMAGE_RESOLUTION_320x240);
		if(yP < yO) degree = 180 - degree;
		break;
	case MOTOR_PULSO:
		break;
	case MOTOR_GARRA:
		break;
	}

	if (degree < 0 || degree > 180) return S_FALSE;
	else {result = ((int) degree); return S_OK;}
}

USHORT getJointDepth(USHORT *pDepth, Vector4 joint){
	FLOAT xF, yF;
	NuiTransformSkeletonToDepthImage(joint, &xF, &yF, NUI_IMAGE_RESOLUTION_320x240);
	int pos = (int) (yF*240.0 + xF);
	return pDepth[pos];
}

#endif