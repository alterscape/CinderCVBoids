/*
 *  BoidController.cpp
 *  Boids
 *
 *  Created by Ryan Spicer on 11/9/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#include "cinder/app/AppBasic.h"
#include "cinder/Rand.h"
#include "cinder/Vector.h"
#include "BoidController.h"

using namespace ci;
using std::list;

BoidController::BoidController()
{
	mPerlin = Perlin( 4 );
}

void BoidController::applyForceToBoids( float zoneRadius, float lowerThresh, float higherThresh, float attractStrength, float repelStrength, float alignStrength  )
{
	float twoPI = M_PI * 2.0f;
	boidCentroid = Vec3f::zero();
	numBoids = particles.size();
	
	for( list<Boid>::iterator p1 = particles.begin(); p1 != particles.end(); ++p1 ){
		
		list<Boid>::iterator p2 = p1;
		for( ++p2; p2 != particles.end(); ++p2 ) {
			Vec3f dir = p1->pos - p2->pos;
			float distSqrd = dir.lengthSquared();
			float zoneRadiusSqrd = zoneRadius * p1->mCrowdFactor * zoneRadius * p2->mCrowdFactor;
			
			if( distSqrd < zoneRadiusSqrd ){		// Neighbor is in the zone
				float per = distSqrd/zoneRadiusSqrd;
				p1->addNeighborPos( p2->pos );
				p2->addNeighborPos( p1->pos );
				
				if( per < lowerThresh ){			// Separation
					float F = ( lowerThresh/per - 1.0f ) * repelStrength;
					dir.normalize();
					dir *= F;
					
					p1->acc += dir;
					p2->acc -= dir;
				} else if( per < higherThresh ){	// Alignment
					float threshDelta	= higherThresh - lowerThresh;
					float adjPer		= ( per - lowerThresh )/threshDelta;
					float F				= ( 1.0 - ( cos( adjPer * twoPI ) * -0.5f + 0.5f ) ) * alignStrength;
					
					p1->acc += p2->velNormal * F;
					p2->acc += p1->velNormal * F;
					
				} else {							// Cohesion (prep)
					float threshDelta	= 1.0f - higherThresh;
					float adjPer		= ( per - higherThresh )/threshDelta;
					float F				= ( 1.0 - ( cos( adjPer * twoPI ) * -0.5f + 0.5f ) ) * attractStrength;
					
					dir.normalize();
					dir *= F;
					
					p1->acc -= dir;
					p2->acc += dir;
				}
			}
		}
		
		boidCentroid += p1->pos;
		/*
		 if( p1->mNumNeighbors > 0 ){ // Cohesion 
		 Vec3f neighborAveragePos = ( p1->mNeighborPos/(float)p1->mNumNeighbors );
		 p1->mAcc += ( neighborAveragePos - p1->mPos ) * attractStrength;	
		 }
		 */
		
		// ADD PERLIN NOISE INFLUENCE
		float scale = 0.002f;
		float multi = 0.01f;
		Vec3f perlin = mPerlin.dfBm( p1->pos * scale ) * multi;
		p1->acc += perlin;
		
		
		// CHECK WHETHER THERE IS ANY PARTICLE/PREDATOR INTERACTION
		/*float eatDistSqrd = 50.0f;
		float predatorZoneRadiusSqrd = zoneRadius * zoneRadius * 5.0f;
		for( list<Predator>::iterator predator = mPredators.begin(); predator != mPredators.end(); ++predator ) {
			
			Vec3f dir = p1->mPos - predator->mPos[0];
			float distSqrd = dir.lengthSquared();
			
			if( distSqrd < predatorZoneRadiusSqrd ){
				if( distSqrd > eatDistSqrd ){
					float F = ( predatorZoneRadiusSqrd/distSqrd - 1.0f ) * 0.1f;
					p1->mFear += F * 0.1f;
					dir = dir.normalized() * F;
					p1->mAcc += dir;
					if( predator->mIsHungry )
						predator->mAcc += dir * 0.04f * predator->mHunger;
				} else {
					p1->mIsDead = true;
					predator->mHunger = 0.0f;
					predator->mIsHungry = false;
				}
			}
		}*/
		
	}
	boidCentroid /= (float)numBoids;
}


void BoidController::applyForceToPredators( float zoneRadius, float lowerThresh, float higherThresh )
{
	/*float twoPI = M_PI * 2.0f;
	for( list<Predator>::iterator P1 = mPredators.begin(); P1 != mPredators.end(); ++P1 ){
		
		list<Predator>::iterator P2 = P1;
		for( ++P2; P2 != mPredators.end(); ++P2 ) {
			Vec3f dir = P1->mPos[0] - P2->mPos[0];
			float distSqrd = dir.lengthSquared();
			float zoneRadiusSqrd = zoneRadius * zoneRadius * 4.0f;
			
			if( distSqrd < zoneRadiusSqrd ){		// Neighbor is in the zone
				float per = distSqrd/zoneRadiusSqrd;
				if( per < lowerThresh ){			// Separation
					float F = ( lowerThresh/per - 1.0f ) * 0.01f;
					dir.normalize();
					dir *= F;
					
					P1->mAcc += dir;
					P2->mAcc -= dir;
				} else if( per < higherThresh ){	// Alignment
					float threshDelta	= higherThresh - lowerThresh;
					float adjPer		= ( per - lowerThresh )/threshDelta;
					float F				= ( 1.0f - cos( adjPer * twoPI ) * -0.5f + 0.5f ) * 0.3f;
					
					P1->mAcc += P2->mVelNormal * F;
					P2->mAcc += P1->mVelNormal * F;
					
				} else {							// Cohesion
					float threshDelta	= 1.0f - higherThresh;
					float adjPer		= ( per - higherThresh )/threshDelta;
					float F				= ( 1.0 - ( cos( adjPer * twoPI ) * -0.5f + 0.5f ) ) * 0.1f;
					
					dir.normalize();
					dir *= F;
					
					P1->mAcc -= dir;
					P2->mAcc += dir;
				}
			}
		}
	}*/
}


void BoidController::pullToCenter( const ci::Vec3f &center )
{
	for( list<Boid>::iterator p = particles.begin(); p != particles.end(); ++p ){
		p->pullToCenter( center );
	}
	
	/*for( list<Predator>::iterator p = mPredators.begin(); p != mPredators.end(); ++p ){
		p->pullToCenter( center );
	}*/
}

void BoidController::update( bool flatten )
{
	for( list<Boid>::iterator p = particles.begin(); p != particles.end(); ){
		if( p->mIsDead ){
			p = particles.erase( p );
		} else {
			p->update( flatten );
			++p;
		}
	}
	
	/*for( list<Predator>::iterator p = mPredators.begin(); p != mPredators.end(); ++p ){
		p->update( flatten );
	}*/
}

void BoidController::draw()
{	
	// DRAW PARTICLE ARROWS
	gl::color( ColorA( 1.0f, 1.0f, 1.0f, 1.0f ) );
	//glBegin( GL_LINES );
	for( list<Boid>::iterator p = particles.begin(); p != particles.end(); ++p ){
		p->draw();
	}
	//glEnd();
}

void BoidController::addPredators( int amt )
{
	/*for( int i=0; i<amt; i++ )
	{
		Vec3f pos = Rand::randVec3f() * Rand::randFloat( 500.0f, 750.0f );
		Vec3f vel = Rand::randVec3f();
		mPredators.push_back( Predator( pos, vel ) );
	}*/
}

void BoidController::addBoids( int amt )
{
	for( int i=0; i<amt; i++ )
	{
		Vec3f pos = Rand::randVec3f() * Rand::randFloat( 100.0f, 200.0f );
		Vec3f vel = Rand::randVec3f();
		
		bool followed = false;
		if( particles.size() == 0 ) followed = true;
		
		particles.push_back( Boid( pos, vel, followed ) );
	}
}

void BoidController::removeBoids( int amt )
{
	for( int i=0; i<amt; i++ )
	{
		particles.pop_back();
	}
}

Vec3f BoidController::getPos()
{
	return particles.begin()->pos;
}
