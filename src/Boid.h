/*
 *  Boid.h
 *  Boids
 *
 *  Created by Ryan Spicer on 11/9/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#pragma once
#include "cinder/Vector.h"
#include "cinder/Color.h"
#include <vector>

class Boid {
public:
	Boid();
	Boid( ci::Vec3f pos, ci::Vec3f vel, bool followed );
	void pullToCenter( const ci::Vec3f &center );
	void update( bool flatten );
	void draw();
	void drawTail();
	void limitSpeed();
	void addNeighborPos( ci::Vec3f pos );
	
	ci::Vec3f	pos;
	ci::Vec3f	tailPos;
	ci::Vec3f	vel;
	ci::Vec3f	velNormal;
	ci::Vec3f	acc;
	
	ci::Vec3f	mNeighborPos;
	int			mNumNeighbors;
	
	ci::Color	mColor;
	
	float		mDecay;
	float		mRadius;
	float		mLength;
	float		mMaxSpeed, mMaxSpeedSqrd;
	float		mMinSpeed, mMinSpeedSqrd;
	float		mFear;
	float		mCrowdFactor;
	
	bool		mIsDead;
	bool		mFollowed;
};