#include "Boid.h"
#include "BoidController.h"
#include "cinder/Rand.h"
#include "cinder/gl/gl.h"
#include "cinder/app/AppBasic.h"

using namespace ci;
using std::vector;

Boid::Boid( Vec3f pos, Vec3f vel, bool followed, BoidController* parent )
{
	this->parent	= parent;
	this->pos		= pos;
	tailPos			= pos;
	this->vel		= vel;
	velNormal		= Vec3f::yAxis();
	this->acc		= Vec3f::zero();
	
	mNeighborPos	= Vec3f::zero();
	mNumNeighbors	= 0;
	mMaxSpeed		= Rand::randFloat( 2.5f, 4.0f );
	mMaxSpeedSqrd	= mMaxSpeed * mMaxSpeed;
	mMinSpeed		= Rand::randFloat( 1.0f, 1.5f );
	mMinSpeedSqrd	= mMinSpeed * mMinSpeed;
	
	mColor			= ColorA( 1.0f, 1.0f, 1.0f, 1.0f );
	
	mDecay			= 0.99f;
	mRadius			= 1.0f;
	mLength			= 5.0f;
	mFear			= 1.0f;
	mCrowdFactor	= 1.0f;
	
	mIsDead			= false;
	mFollowed		= followed;
	drawClosestSilhouettePoint = true;
}

void Boid::pullToCenter( const Vec3f &center )
{
	Vec3f dirToCenter = pos - center;
	float distToCenter = dirToCenter.length();
	float distThresh = 200.0f;
	
	if( distToCenter > distThresh ){
		dirToCenter.normalize();
		float pullStrength = 0.00025f;
		vel -= dirToCenter * ( ( distToCenter - distThresh ) * pullStrength );
	}
}


void Boid::update( bool flatten )
{	
	mCrowdFactor -= ( mCrowdFactor - ( 1.0f - mNumNeighbors * 0.02f ) ) * 0.1f;
	mCrowdFactor = constrain( mCrowdFactor, 0.5f, 1.0f );
	
	mFear -= ( mFear) * 0.2f;
	
	if( flatten )
		acc.z = 0.0f;
	
	vel += acc;
	velNormal = vel.normalized();
	
	limitSpeed();
	
	
	pos += vel;
	if( flatten )
		pos.z = 0.0f;
	
	tailPos = pos - ( velNormal * mLength );
	vel *= mDecay;
	
	//float c = math<float>::min( mNumNeighbors/50.0f, 1.0f );
//	mColor = ColorA( CM_HSV, 1.0f - c, c, c * 0.5f + 0.5f, 1.0f );
	mColor = parent->getColor(this);
	
	acc = Vec3f::zero();
	mNeighborPos = Vec3f::zero();
	mNumNeighbors = 0;
}

void Boid::limitSpeed()
{
	float maxSpeed = mMaxSpeed + mCrowdFactor;
	float maxSpeedSqrd = maxSpeed * maxSpeed;
	
	float vLengthSqrd = vel.lengthSquared();
	if( vLengthSqrd > maxSpeedSqrd ){
		vel = velNormal * maxSpeed;
		
	} else if( vLengthSqrd < mMinSpeedSqrd ){
		vel = velNormal * mMinSpeed;
	}
	vel *= (1.0 + mFear );
}

void Boid::draw()
{
	glColor4f( mColor );
	gl::drawVector( pos - velNormal * mLength, pos - velNormal * mLength * 0.75f, mLength * 0.7f, mRadius );
	if(drawClosestSilhouettePoint) {
		glColor3f(0.0f,1.0f,0.0f);
		glBegin(GL_LINES);
		glLineWidth(1.0f);
		gl::vertex(pos);
		gl::vertex(closestSilhouettePoint);
		glEnd();
	}
	
}

void Boid::drawTail()
{
	glColor4f( mColor );
	glVertex3fv( pos );
	glColor4f( ColorA( mColor.r, mColor.g, mColor.b, 0.01f ) );
	glVertex3fv( tailPos );
}

void Boid::addNeighborPos( Vec3f pos )
{
	mNeighborPos += pos;
	mNumNeighbors ++;
}


