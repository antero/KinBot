#ifndef SKELETON_ANGLES

#define SKELETON_ANGLES

#define PI 3.14159265

#include <Windows.h>
#include <NuiApi.h>
#include <math.h>

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
HRESULT threeJointAngle(Vector4 j1, Vector4 j2, Vector4 j3, int &result){
	Vector4 braco, antebraco;
	vecsub(j1, j2, braco);
	vecsub(j3, j2, antebraco);
	float cosTeta = dotproduct(braco, antebraco) / (module(braco) * module(antebraco)) ;
	float degree = acos(cosTeta) * 180.0 / PI;
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