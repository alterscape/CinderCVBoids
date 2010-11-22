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

class BoidController {
public:
	BoidController();
	void applyForceToBoids();// float zoneRadius, float lowerThresh, float higherThresh, float attractStrength, float repelStrength, float orientStrength );
	void pullToCenter( const ci::Vec3f &center );
	void update();
	void draw();
	void addBoids( int amt );
	void removeBoids( int amt );
	ci::Vec3f getPos();
	
	//I don't like exposing these this way, but it makes mParams happier;
	float	zoneRadius;
	float	lowerThresh;
	float	higherThresh;
	float	attractStrength;
	float	repelStrength;
	float	orientStrength;
	
	bool	centralGravity;
	bool	flatten;
	
private:
	
	ci::Perlin mPerlin;
	
	std::list<Boid>	particles;
	std::list<BoidController> otherControllers;
	ci::Vec3f boidCentroid;
	int numBoids;
	
	static const float TWO_PI = M_PI * 2.0f;
};