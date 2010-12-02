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
using namespace std;

BoidController::BoidController()
{
	mPerlin = Perlin( 4 );
	zoneRadius			= 80.0f;
	lowerThresh			= 0.5f;
	higherThresh		= 0.8f;
	attractStrength		= 0.004f;
	repelStrength		= 0.01f;
	orientStrength		= 0.01f;
	
	centralGravity		= true;
	flatten				= true;	
}

void BoidController::applyForceToBoids()
{
	if(lowerThresh > higherThresh ) higherThresh = lowerThresh;
	
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
					float F				= ( 1.0 - ( cos( adjPer * TWO_PI ) * -0.5f + 0.5f ) ) * orientStrength;
					
					p1->acc += p2->velNormal * F;
					p2->acc += p1->velNormal * F;
					
				} else {							// Cohesion (prep)
					float threshDelta	= 1.0f - higherThresh;
					float adjPer		= ( per - higherThresh )/threshDelta;
					float F				= ( 1.0 - ( cos( adjPer * TWO_PI ) * -0.5f + 0.5f ) ) * attractStrength;
					
					dir.normalize();
					dir *= F;
					
					p1->acc -= dir;
					p2->acc += dir;
				}
			}			
		}
		
		BoidController *controller = otherControllers.front();
		
		list<Boid> *otherParticles = &controller->particles;
		for (list<Boid>::iterator p3 = otherParticles->begin(); p3 != otherParticles->end(); ++p3) {
			Vec3f dir = p1->pos - p3->pos;
			float distSqrd = dir.lengthSquared();
			float zoneRadiusSqrd = zoneRadius * p1->mCrowdFactor * zoneRadius * p3->mCrowdFactor;
			
			if( distSqrd < zoneRadiusSqrd ){		// Neighbor is in the zone
				float per = distSqrd/zoneRadiusSqrd;
				p1->addNeighborPos( p3->pos );
				p3->addNeighborPos( p1->pos );
				
				if( per < lowerThresh ){			// Separation
					float F = ( lowerThresh/per - 1.0f ) * repelStrength;
					dir.normalize();
					dir *= F;
					
					p1->acc += dir;
					p3->acc -= dir;
				} else if( per < higherThresh ){	// Alignment
					float threshDelta	= higherThresh - lowerThresh;
					float adjPer		= ( per - lowerThresh )/threshDelta;
					float F				= ( 1.0 - ( cos( adjPer * TWO_PI ) * -0.5f + 0.5f ) ) * orientStrength;
					
					p1->acc += p3->velNormal * F;
					p3->acc += p1->velNormal * F;
					
				} else {							// Cohesion (prep)
					float threshDelta	= 1.0f - higherThresh;
					float adjPer		= ( per - higherThresh )/threshDelta;
					float F				= ( 1.0 - ( cos( adjPer * TWO_PI ) * -0.5f + 0.5f ) ) * attractStrength;
					
					dir.normalize();
					dir *= F;
					
					p1->acc -= dir;
					p3->acc += dir;
				}
			}
		}
	
		boidCentroid += p1->pos;
		
		//respond to silhouettes
		//for( CvSeq* c=silhouettes; c!=NULL; c=c->h_next ){
			//do nothing yet
		//}
		
		if( p1->mNumNeighbors > 0 ){ // Cohesion 
			Vec3f neighborAveragePos = ( p1->mNeighborPos/(float)p1->mNumNeighbors );
			p1->acc += ( neighborAveragePos - p1->pos ) * attractStrength;	
		}
		
		// ADD PERLIN NOISE INFLUENCE
		float scale = 0.005f;
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

void BoidController::applySilhouetteToBoids(std::vector<Vec2i_ptr_vec> * polygons, ci::Matrix44<float> *imageToWorldMap)
{
	Matrix44<float> worldToImage = Matrix44<float>(*imageToWorldMap);
	worldToImage.invert();
	for( list<Boid>::iterator p1 = particles.begin(); p1 != particles.end(); ++p1 ){	//for each boid
		Vec3f xformedPos = worldToImage.transformVec(p1->pos);
				xformedPos.z = 0.0f;	//Force Z to be 0 for these calculations.
		float closestDistanceSquared = 9999999.9f;
		Vec3f closestPoint;
		cout << "checking " << polygons->size() << " polygons!" << endl; 
		for(vector<Vec2i_ptr_vec>::iterator polygon = polygons->begin(); polygon!=polygons->end();++polygon) {
			cout<< "checking a polygon with " << polygon->get()->size() << " points!" << endl;
			//the two-iterator plan, as per http://stackoverflow.com/questions/261271/compare-two-consecutive-elements-in-stdlist
			vector<Vec2i_ptr>::iterator secondPoint = polygon->get()->begin(), end = polygon->get()->end();
			if (secondPoint != end) {
				for(vector<Vec2i_ptr>::iterator firstPoint = secondPoint++;
					secondPoint!=end;
					++firstPoint, ++secondPoint) 
				{
					cout << "checking a pair of points!" << endl;
					//glVertex2f(point->get()->x,point->get()->y);
					ci::Vec3f pt1 = ci::Vec3f(firstPoint->get()->x,firstPoint->get()->y,0.0f);
					ci::Vec3f pt2 = ci::Vec3f(secondPoint->get()->x,secondPoint->get()->y,0.0f);
					ci::Vec3f distance = getClosestPointToSegment(&pt1,&pt2,&xformedPos);
					if(distance.lengthSquared() < closestDistanceSquared) {
						cout << "this point is closer!" << endl;
						closestDistanceSquared = distance.lengthSquared();
						closestPoint = distance;
					}
				}
			}
		}
		float silThresh = 1000.0f;
		float repelStrength = 5.0f;
		if (closestDistanceSquared < silThresh) {	//FIXME magic numbers suck
			float per = closestDistanceSquared/silThresh;
			Vec3f distance = xformedPos-closestPoint;
			cout << "original pos: (" << p1->pos.x << "," << p1->pos.y << "," << p1->pos.z << "; xformed pos: (" << xformedPos.x << "," << xformedPos.y << "," << xformedPos.z << "); distance to closest^2: " << closestDistanceSquared <<endl;
			std::cout << "distance, for example: " << distance.length() << "; point: (" << closestPoint.x << "," << closestPoint.y << ")" << std::endl;
			
			float F = ( 1.0f - per ) * repelStrength;
			distance.normalize();
			distance *= F;
			std::cout << "repelling from silhouette with vector: (" << distance.x << ","<< distance.y << "," << distance.z << "); magnitude: " << F << std::endl;
			p1->acc += distance;
		}
		
	}
}


void BoidController::pullToCenter( const ci::Vec3f &center )
{
	for( list<Boid>::iterator p = particles.begin(); p != particles.end(); ++p ){
		p->pullToCenter( center );
	}
	
}

void BoidController::update()
{
	for( list<Boid>::iterator p = particles.begin(); p != particles.end(); ){
		if( p->mIsDead ){
			p = particles.erase( p );
		} else {
			p->update( flatten );
			++p;
		}
	}
	
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


void BoidController::addBoids( int amt )
{
	for( int i=0; i<amt; i++ )
	{
		Vec3f pos = Rand::randVec3f() * Rand::randFloat( 100.0f, 200.0f );
		Vec3f vel = Rand::randVec3f();
		
		bool followed = false;
		if( particles.size() == 0 ) followed = true;
		
		particles.push_back( Boid( pos, vel, followed, this ) );
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

/**
 * Returns the color for a boid belonging to this boid controller.
 * @param boid the boid to operate on
 * @return the color the boid should be painted.
 */

ci::Color BoidController::getColor(Boid *boid) 
{
//	float c = math<float>::min( boid->mNumNeighbors/50.0f, 1.0f );
//	return ColorA( CM_HSV, 1.0f - c, c, c * 0.5f + 0.5f, 1.0f );
	return baseColor;
}

void BoidController::addOtherFlock(BoidController *flock)
{
	otherControllers.push_back(flock);
}


