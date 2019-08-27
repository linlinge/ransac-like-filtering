#pragma once
#include "V3.hpp"
#include <iostream>
using namespace std;

enum LineInitType{PD,PP};
enum ProjectionType{XY,XZ,YZ};
class Plane;

/* 
	Pointwise Equation:
 		(x-x1)/d.x=(y-y1)/d.y=(z-z1)/d.z
*/
class Line
{
public:
	// represent by parameter	
	V3 Point_,Direction_;
	
	// convert point-direction equation to parameter equation
	Line(V3 dat1, V3 dat2, LineInitType mode);
	V3 IsIntersect(Plane& dat);
	V3 GetProjectionVector(ProjectionType mode);
	float GetProjectionArc(ProjectionType mode);
	V3 TransformTo(ProjectionType mode);
};

class Plane
{
public:
	float A_, B_, C_, D_;
	Plane(V3 P1, V3 P2, V3 P3);
	V3 IsIntersect(Line& dat);
	
	float Distance(V3 pt){
		float numerator=abs(A_*pt.x+B_*pt.y+C_*pt.z+D_);
		float denominator=sqrt(pow(A_,2)+pow(B_,2)+pow(C_,2));
		return numerator/denominator;
	}	
	
	friend std::ostream& operator <<(std::ostream& os,Plane& dat)
	{
		os<<dat.A_<<" "<<dat.B_<<" "<<dat.C_<<" "<<dat.D_;
		return os;
	}
};

class Angle
{
public:
	float arc_, angle_;	
	Angle(V3& mid,V3& left,V3& right);
};

