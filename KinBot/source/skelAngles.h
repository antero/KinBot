#ifndef SKELETON_ANGLES

#define SKELETON_ANGLES

#include <Windows.h>
#include <NuiApi.h>
#include <math.h>
#include <stdio.h>

#define SERVO_BASE				1
#define SERVO_SHOULDER_RIGHT	2
#define SERVO_ELBOW_RIGHT		3
#define SERVO_WRIST_RIGHT		4
#define SERVO_HAND_RIGHT		5

#define PI 3.14159265
#define posi(x,y) x+y*320


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

//return angle between two vectors
HRESULT twoVectorAngle(Vector4 v1, Vector4 v2, int motor, int &result){
	float cosTeta = dotproduct(v1, v2) / (module(v1) * module(v2)) ;
	float degree = acos(cosTeta)* 180.0 / PI;
	switch (motor){
	case SERVO_BASE:
	case SERVO_SHOULDER_RIGHT:
		break;
	case SERVO_ELBOW_RIGHT:
		degree = 180-degree;
		degree = 100-degree;
		break;
	case SERVO_WRIST_RIGHT:
		break;
	case SERVO_HAND_RIGHT:
	default:
		break;
	}
	if (degree < 0 || degree > 180) return S_FALSE;
	else {result = ((int) degree); return S_OK;}
}

//return angle between three joints
HRESULT threeJointAngle(Vector4 j1, Vector4 j2, Vector4 j3, int motor, int &result){
	Vector4 v1, v2;
	vecsub(j1, j2, v1);
	vecsub(j3, j2, v2);
	return twoVectorAngle(v1, v2, motor, result);
}

void getJointDepth(USHORT *pDepth, Vector4 joint, USHORT *positions){
	FLOAT xF, yF;
	NuiTransformSkeletonToDepthImage(joint, &xF, &yF, NUI_IMAGE_RESOLUTION_320x240);
	int x = (int) xF;
	int y = (int) yF;
	int loc = -1;
	for(int i=-1; i<2;i++){
		for(int j=-1;j<2;j++){
			loc++;
			if( x+i<0 || x+i>319){positions[loc] =-1; continue;}
			if( y+j<0 || y+j>239) {positions[loc] =-1; continue;}
			positions[loc] = NuiDepthPixelToDepth(pDepth[posi((x+i),(y+j))]);
		}
	}
}

#endif