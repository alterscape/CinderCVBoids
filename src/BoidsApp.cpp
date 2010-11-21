#include "cinder/app/AppBasic.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "cinder/ImageIO.h"
#include "cinder/params/Params.h"
#include "cinder/Camera.h"
#include "cinder/Rand.h"
#include "BoidController.h"

//CV stuff
#include "cinder/gl/Texture.h"
#include "cinder/Capture.h"
#include "CinderOpenCV.h"

#include <vector>

#define NUM_INITIAL_PARTICLES 500
#define NUM_INITIAL_PREDATORS 9
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
	
	// PARAMS
	params::InterfaceGl	mParams;
	
	// CAMERA
	CameraPersp			mCam;
	Quatf				mSceneRotation;
	Vec3f				mEye, mCenter, mUp;
	float				mCameraDistance;
	
	BoidController		boidController;
	float				mZoneRadius;
	float				mLowerThresh, mHigherThresh;
	float				mAttractStrength, mRepelStrength, mOrientStrength;
	
	bool				mCentralGravity;
	bool				mFlatten;
	bool				mSaveFrames;
	bool				mIsRenderingPrint;
	
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
	mCentralGravity		= true;
	mFlatten			= false;
	mSaveFrames			= false;
	mIsRenderingPrint	= false;
	mZoneRadius			= 80.0f;
	mLowerThresh		= 0.5f;
	mHigherThresh		= 0.8f;
	mAttractStrength	= 0.004f;
	mRepelStrength		= 0.01f;
	mOrientStrength		= 0.01f;
	
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
	
	// SETUP PARAMS
	mParams = params::InterfaceGl( "Flocking", Vec2i( 200, 310 ) );
	mParams.addParam( "Scene Rotation", &mSceneRotation, "opened=1" );
	mParams.addSeparator();
	mParams.addParam( "Eye Distance", &mCameraDistance, "min=100.0 max=2000.0 step=50.0 keyIncr=s keyDecr=w" );
	mParams.addParam( "Center Gravity", &mCentralGravity, "keyIncr=g" );
	mParams.addParam( "Flatten", &mFlatten, "keyIncr=f" );
	mParams.addSeparator();
	mParams.addParam( "Zone Radius", &mZoneRadius, "min=10.0 max=100.0 step=1.0 keyIncr=z keyDecr=Z" );
	mParams.addParam( "Lower Thresh", &mLowerThresh, "min=0.025 max=1.0 step=0.025 keyIncr=l keyDecr=L" );
	mParams.addParam( "Higher Thresh", &mHigherThresh, "min=0.025 max=1.0 step=0.025 keyIncr=h keyDecr=H" );
	mParams.addSeparator();
	mParams.addParam( "Attract Strength", &mAttractStrength, "min=0.001 max=0.1 step=0.001 keyIncr=a keyDecr=A" );
	mParams.addParam( "Repel Strength", &mRepelStrength, "min=0.001 max=0.1 step=0.001 keyIncr=r keyDecr=R" );
	mParams.addParam( "Orient Strength", &mOrientStrength, "min=0.001 max=0.1 step=0.001 keyIncr=o keyDecr=O" );
	mParams.addSeparator();
	mParams.addParam( "CV Threshhold", &cvThreshholdLevel, "min=0 max=255 step=1 keyIncr=t keyDecr=T" );
	
	// CREATE PARTICLE CONTROLLER
	boidController.addBoids( NUM_INITIAL_PARTICLES );
	//boidController.addPredators( NUM_INITIAL_PREDATORS );
}

void BoidsApp::keyDown( KeyEvent event )
{
	if( event.getChar() == 'p' ){
		boidController.addBoids( NUM_PARTICLES_TO_SPAWN );
	} else if( event.getChar() == ' ' ){
		mSaveFrames = !mSaveFrames;
	}
}


void BoidsApp::update()
{
	if( mLowerThresh > mHigherThresh ) mHigherThresh = mLowerThresh;
	
	//boidController.applyForceToPredators( mZoneRadius, 0.4f, 0.7f );	
	boidController.applyForceToBoids( mZoneRadius, mLowerThresh, mHigherThresh, mAttractStrength, mRepelStrength, mOrientStrength );
	if( mCentralGravity ) boidController.pullToCenter( mCenter );
	boidController.update( mFlatten );
	
	mEye	= Vec3f( 0.0f, 0.0f, mCameraDistance );
	mCam.lookAt( mEye, mCenter, mUp );
	gl::setMatrices( mCam );
	gl::rotate( mSceneRotation );
	
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
	boidController.draw();
	
	if( mSaveFrames ){
		writeImage( getHomeDirectory() + "flocking/image_" + toString( getElapsedFrames() ) + ".png", copyWindowSurface() );
	}
	
	// DRAW PARAMS WINDOW
	params::InterfaceGl::draw();
}



CINDER_APP_BASIC( BoidsApp, RendererGl )
