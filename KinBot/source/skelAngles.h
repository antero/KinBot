#ifndef SKELETON_ANGLES

#define SKELETON_ANGLES

#define PI 3.14159265

#include <Windows.h>
#include <NuiApi.h>
#include <math.h>
#include <stdio.h>

#define MOTOR_BASE		1
#define MOTOR_OMBRO		2
#define MOTOR_COTOVELO  3
#define MOTOR_PULSO		4
#define MOTOR_GARRA		5
#define posi(x,y) x+y*240

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
HRESULT twoVectorAngle(Vector4 v1, Vector4 v2, int motor, int &result){
	float cosTeta = dotproduct(v1, v2) / (module(v1) * module(v2)) ;
	float degree = acos(cosTeta)* 180.0 / PI;
	switch (motor){
	case MOTOR_BASE:
	case MOTOR_OMBRO:
		break;
	case MOTOR_COTOVELO:
		degree = 180-degree;
		break;
	case MOTOR_PULSO:
		//if (degree < 0 || degree > 90) degree = 500;
		break;
	case MOTOR_GARRA:
	default:
		break;
	}
	if (degree < 0 || degree > 180) return S_FALSE;
	else {result = ((int) degree); return S_OK;}
}

HRESULT threeJointAngle(Vector4 j1, Vector4 j2, Vector4 j3, int motor, int &result){
	Vector4 v1, v2;
	vecsub(j1, j2, v1);
	vecsub(j3, j2, v2);
	return twoVectorAngle(v1, v2, motor, result);
}

void getJointDepth(USHORT *pDepth, Vector4 joint, int &x, int &y, USHORT *positions){
	FLOAT xF, yF;
	NuiTransformSkeletonToDepthImage(joint, &xF, &yF, NUI_IMAGE_RESOLUTION_320x240);
	x = (int) xF;
	y = (int) yF;
	int loc = -1;
	for(int i=-1; i<2;i++){
		loc++;
		if( x+i<0 || x+i>319) continue;
		for(int j=-1;j<2;j++){
			if( y+j<0 || y+j>239) continue;
			positions[loc] = NuiDepthPixelToDepth(pDepth[posi((x+i),(y+j))]);
		}
	}
	//return NuiDepthPixelToDepth(pDepth[pos]);
}

#endif