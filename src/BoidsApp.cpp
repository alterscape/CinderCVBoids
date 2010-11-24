#include "cinder/app/AppBasic.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "cinder/ImageIO.h"
#include "cinder/params/Params.h"
#include "cinder/Camera.h"
#include "cinder/Rand.h"
#include "BoidController.h"
#include "time.h"

//CV stuff
#include "cinder/gl/Texture.h"
#include "cinder/Capture.h"
#include "CinderOpenCV.h"

#include <vector>

#define NUM_INITIAL_PARTICLES 120
#define NUM_PARTICLES_TO_SPAWN 15

using namespace ci;
using namespace ci::app;

class BoidsApp : public AppBasic {
public:
	void prepareSettings( Settings *settings );
	void keyDown( KeyEvent event );
	void setup();
	void update();
	void draw();
	bool checkTime();
	
	// PARAMS
	params::InterfaceGl	mParams;
	
	// CAMERA
	CameraPersp			mCam;
	Quatf				mSceneRotation;
	Vec3f				mEye, mCenter, mUp;
	float				mCameraDistance;
	clock_t				changeTimer;
	BoidController		flock_one;
	BoidController		flock_two;

	bool				mSaveFrames;
	bool				mIsRenderingPrint;
	
	float				changeInterval;
	
	Capture				capture;
	gl::Texture			texture;
	int					cvThreshholdLevel;
};

void BoidsApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 875, 600 );
	settings->setFrameRate( 60.0f );
}

void BoidsApp::setup()
{	
	Rand::randomize();
	
	mCenter				= Vec3f( getWindowWidth() * 0.5f, getWindowHeight() * 0.5f, 0.0f );
	mSaveFrames			= false;
	mIsRenderingPrint	= false;

	changeInterval		= 5.0;
	changeTimer			= clock() + changeInterval * CLOCKS_PER_SEC;
	
	// SETUP CAMERA
	mCameraDistance		= 350.0f;
	mEye				= Vec3f( 0.0f, 0.0f, mCameraDistance );
	mCenter				= Vec3f::zero();
	mUp					= Vec3f::yAxis();
	mCam.setPerspective( 75.0f, getWindowAspectRatio(), 5.0f, 5000.0f );
	
	// Initialize the OpenCV input (Below added RS 2010-11-15)
	try {
		//ci::Device device = Capture.
		std::vector<boost::shared_ptr<Capture::Device> > devices = Capture::getDevices();
		
		std::cout << "Capture Devices:" << std::endl;
		std::vector<boost::shared_ptr<Capture::Device> >::iterator device_iterator;
		for(device_iterator=devices.begin(); device_iterator!=devices.end(); device_iterator++)
		{
			console() << device_iterator->get()->getName() << std::endl;
		}
		
		capture = Capture(320,240);
		capture.start();
	} catch ( ... ) {
		console() << "Failed to initialize capture device" << std::endl;
	}
	cvThreshholdLevel = 128;
		
	// CREATE PARTICLE CONTROLLER
	flock_one.addBoids( NUM_INITIAL_PARTICLES );
	flock_two.addBoids( NUM_INITIAL_PARTICLES );
	//flock_one.baseColor = ColorA( CM_HSV, 0, 0.19, 0.23);		//Muted red
	//flock_two.baseColor = ColorA( CM_HSV, 0.94, 0.26, 0.23);	//Muted blue
	flock_one.baseColor = ColorA( CM_RGB, 1.0, 0.0, 0.0);
	flock_two.baseColor = ColorA( CM_RGB, 0.0, 0.0, 1.0);
	
	flock_one.addOtherFlock(&flock_two);
	flock_two.addOtherFlock(&flock_one);
	// SETUP PARAMS
	mParams = params::InterfaceGl( "Flocking", Vec2i( 200, 310 ) );
	mParams.addParam( "Scene Rotation", &mSceneRotation, "opened=1" );
	mParams.addSeparator();
	mParams.addParam( "Eye Distance", &mCameraDistance, "min=100.0 max=2000.0 step=50.0 keyIncr=s keyDecr=w" );
	mParams.addParam( "Center Gravity", &flock_one.centralGravity, "keyIncr=g" );
	mParams.addParam( "Flatten", &flock_one.flatten, "keyIncr=f" );
	mParams.addSeparator();
	mParams.addParam( "Zone Radius", &flock_one.zoneRadius, "min=10.0 max=100.0 step=1.0 keyIncr=z keyDecr=Z" );
	mParams.addParam( "Lower Thresh", &flock_one.lowerThresh, "min=0.025 max=1.0 step=0.025 keyIncr=l keyDecr=L" );
	mParams.addParam( "Higher Thresh", &flock_one.higherThresh, "min=0.025 max=1.0 step=0.025 keyIncr=h keyDecr=H" );
	mParams.addSeparator();
	mParams.addParam( "Attract Strength", &flock_one.attractStrength, "min=0.001 max=0.1 step=0.001 keyIncr=a keyDecr=A" );
	mParams.addParam( "Repel Strength", &flock_one.repelStrength, "min=0.001 max=0.1 step=0.001 keyIncr=r keyDecr=R" );
	mParams.addParam( "Orient Strength", &flock_one.orientStrength, "min=0.001 max=0.1 step=0.001 keyIncr=o keyDecr=O" );
	mParams.addSeparator();
	mParams.addParam( "CV Threshhold", &cvThreshholdLevel, "min=0 max=255 step=1 keyIncr=t keyDecr=T" );
	
}

void BoidsApp::keyDown( KeyEvent event )
{
	if( event.getChar() == 'p' ){
		flock_one.addBoids( NUM_PARTICLES_TO_SPAWN );
	} else if( event.getChar() == ' ' ){
		mSaveFrames = !mSaveFrames;
	}
}


void BoidsApp::update()
{
	flock_one.applyForceToBoids();
	if( flock_one.centralGravity ) flock_one.pullToCenter( mCenter );
	flock_one.update();
	
	flock_two.applyForceToBoids();
	if( flock_two.centralGravity) flock_two.pullToCenter( mCenter);
	flock_two.update();
	
	mEye	= Vec3f( 0.0f, 0.0f, mCameraDistance );
	mCam.lookAt( mEye, mCenter, mUp );
	gl::setMatrices( mCam );
	gl::rotate( mSceneRotation );
	
	//if (checkTime()) {
//			flatten = !flatten;
//	}
	
	//OpenCV IO
	if( capture && capture.checkNewFrame() ) {
		cv::Mat input( toOcv( capture.getSurface() ) ), gray, output;
		cv::cvtColor(input,gray,CV_RGB2GRAY);
		cv::threshold( gray, output, cvThreshholdLevel, 255, CV_8U );
		
		ci::Surface surface = fromOcv(output);
		
		ci::Surface::Iter pixelIterator = surface.getIter(Area(0,0,10,10));
		//for (pixelIterator; pixelIterator != pixelIterator.; <#increment#>) {
//			statements
//		}
		//std::vector<boost::shared_ptr<Capture::Device> >::iterator device_iterator;
//		for(device_iterator=devices.begin(); device_iterator!=devices.end(); device_iterator++)
//		{
//			console() << device_iterator->get()->getName() << std::endl;
//		}
		
		texture = gl::Texture( fromOcv( output ) );
	}	 
	
}

void BoidsApp::draw()
{	
	gl::clear( Color( 0, 0, 0 ), true );
	gl::enableDepthRead();
	gl::enableDepthWrite();
	
	gl::color( ColorA( 1.0f, 1.0f, 1.0f, 1.0f) );
	if( texture )
		gl::draw( texture );
	
	gl::color( ColorA( 1.0f, 1.0f, 1.0f, 1.0f ) );
	flock_one.draw();
	
	gl::color( ColorA( 1.0f, 1.0f, 1.0f, 1.0f ) );
	flock_two.draw();
	
	if( mSaveFrames ){
		writeImage( getHomeDirectory() + "flocking/image_" + toString( getElapsedFrames() ) + ".png", copyWindowSurface() );
	}
	
	// DRAW PARAMS WINDOW
	params::InterfaceGl::draw();
}

bool BoidsApp::checkTime()
{	if ( clock() % changeTimer < 1.0 )
		return TRUE;
	else {
		return FALSE;
	}
}


CINDER_APP_BASIC( BoidsApp, RendererGl )
