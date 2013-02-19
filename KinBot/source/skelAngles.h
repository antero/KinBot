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
int threeJointAngle(Vector4 j1, Vector4 j2, Vector4 j3){
	Vector4 braco, antebraco;
	vecsub(j1, j2, braco);
	vecsub(j3, j2, antebraco);
	float cosTeta = dotproduct(braco, antebraco) / (module(braco) * module(antebraco)) ;
	float degree = acos(cosTeta) * 180.0 / PI;
	if (degree < 0 ) return 0;
	else if (degree > 180) return 180;
	else return ((int) degree);
}


#endif