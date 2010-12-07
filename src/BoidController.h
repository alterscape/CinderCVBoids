/*
 *  BoidController.h
 *  Boids
 *
 *  Created by Ryan Spicer on 11/9/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#pragma once
#include "Boid.h"
#include <list>
#include <math.h>
#include "cinder/Perlin.h"
#include "CinderOpenCV.h"
#include "cinder/Cinder.h"
#include "cinder/Matrix.h"
#include <boost/ptr_container/ptr_list.hpp>
#include <vector>
#include "SilhouetteDetector.h"

class BoidController {
public:
	BoidController();
	void applyForceToBoids();// float zoneRadius, float lowerThresh, float higherThresh, float attractStrength, float repelStrength, float orientStrength );
	void applySilhouetteToBoids(std::vector<Vec2i_ptr_vec> *polygons, ci::Matrix44<float> *imageToWorldMap);
	void pullToCenter( const ci::Vec3f &center );
	void update();
	void draw();
	void addBoids( int amt );
	void removeBoids( int amt );
	ci::Color getColor(Boid *boid);
	ci::Vec3f getPos();
	void addOtherFlock(BoidController *flock);
	
	//I don't like exposing these this way, but it makes mParams happier;
	float	zoneRadius;
	float	lowerThresh;
	float	higherThresh;
	float	attractStrength;
	float	repelStrength;
	float	orientStrength;
	
	bool	centralGravity;
	bool	flatten;
	
	ci::Color baseColor;
	
	// mouse
	bool				mMousePressed;
	ci::Vec3f				mousePos;
	
private:
	
	ci::Perlin mPerlin;
	
	std::list<Boid>	particles;
	//boost::ptr_list<BoidController> otherControllers;
	std::list<BoidController*> otherControllers;
	ci::Vec3f boidCentroid;
	int numBoids;
	
	static const float TWO_PI = M_PI * 2.0f;
};

inline float invSqrt(float x){
	float xhalf = 0.5f * x;
	int i = *(int*)&x; // store floating-point bits in integer
	i = 0x5f3759d5 - (i >> 1); // initial guess for Newton's method
	x = *(float*)&i; // convert new bits into float
	x = x*(1.5f - xhalf*x*x); // One round of Newton's method
	return x;
}

/*
 float DistanceToPointSquared(float X, float Y)
 {
	 float vx = x1-X, vy = y1-Y, ux = x2-x1, uy = y2-y1;
	 float length = ux*ux+uy*uy;
	 
	 float det = (-vx*ux)+(-vy*uy); //if this is < 0 or > length then its outside the line segment
	 if(det<0 || det>length)
	 {
		 ux=x2-X;
		 uy=y2-Y;
		 return std::min<float>(vx*vx+vy*vy, ux*ux+uy*uy);
	 }
	 
	 det = ux*vy-uy*vx;
	 return (det*det)/length;
 }
 */

inline float pointLineDistance(CvPoint *e1, CvPoint *e2, ci::Vec3f *p) {
	float length = pow(pow(e1->x+e2->x,2)+pow(e1->y+e2->y,2),0.5);
	float vx = e1->x-p->x;
	float vy = e1->y-p->y;
	float ux = e2->x-e1->x;
	float uy = e2->y-e1->y;
	float det = (-vx*ux)+(-vy*uy); //if this is < 0 or > length then its outside the line segment
	if(det<0 || det>length)
	{
		ux=e2->x-p->x;
		uy=e2->y-p->y;
		return std::min<float>(vx*vx+vy*vy, ux*ux+uy*uy);
	}
	
	det = ux*vy-uy*vx;
	return (det*det)/length;
	
	
}

//we should test this function and make sure Ryan did not do something very silly here.
inline ci::Vec3f getClosestPointToSegment(ci::Vec3f *p1, ci::Vec3f *p2, ci::Vec3f *p)
{
	ci::Vec3f direction = *p2-*p1;
	//std::cout << "p1: ("<<p1->x<<","<<p1->y<<","<<p1->z<<")";
//	std::cout << "; p2: ("<<p2->x<<","<<p2->y<<","<<p2->z<<")";
//	std::cout << "; p: ("<<p->x<<","<<p->y<<","<<p->z<<")";
//	std::cout << "; direction: " << direction.x << "," << direction.y << "," << direction.z << ")";
	ci::Vec3f w = *p-*p1;
	//std::cout << "; w: (" << w.x << "," << w.y << "," << w.z << "); ";
	float proj = w.dot(direction);
	//std::cout << "proj: " << proj;
	if(proj <=0) {
		//std::cout << "; closest point: (" << p1->x << "," << p1->y << "," << p1->z << ")" << std::endl;
		return *p1;
	} else {
		float vsq = direction.dot(direction);
		//std::cout << "; vsq: " << vsq;
		if (proj >= vsq) {
			//ci::Vec3f retVal = *p1+direction;
//			std::cout << "; closest point: (" << retVal.x << "," << retVal.y << "," << retVal.z << ")" << std::endl;
			return *p1+direction;
		} else {
			//ci::Vec3f retVal = *p1 + (proj/vsq)*direction;
//			std::cout << "; closest point: (" << retVal.x << "," << retVal.y << "," << retVal.z << ")" << std::endl;
			return *p1 + (proj/vsq)*direction;
		}
	}
	
}
//end potential silliness
