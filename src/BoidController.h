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
#include "cinder/Perlin.h"
#include <boost/ptr_container/ptr_list.hpp>


class BoidController {
public:
	BoidController();
	void applyForceToBoids();// float zoneRadius, float lowerThresh, float higherThresh, float attractStrength, float repelStrength, float orientStrength );
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