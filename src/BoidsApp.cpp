#include "cinder/app/AppBasic.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "cinder/ImageIO.h"
#include "cinder/params/Params.h"
#include "cinder/Camera.h"
#include "cinder/Rand.h"
#include "BoidController.h"
#include "SilhouetteDetector.h"
#include "time.h"
#include "cinder/Surface.h"

//CV stuff
#include "cinder/gl/Texture.h"
#include "cinder/Capture.h"
#include "CinderOpenCV.h"

#include <vector>

#define NUM_INITIAL_PARTICLES 50
#define NUM_PARTICLES_TO_SPAWN 15

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace boost;

class BoidsApp : public AppBasic {
public:
	void prepareSettings( Settings *settings );
	void keyDown( KeyEvent event );
	void setup();
	void update();
	void drawCapture();
	void draw();
	bool checkTime();
	
	//Mouse code ///
	void mouseDown( MouseEvent event );
	void mouseDrag( MouseEvent event );
	void mouseUp( MouseEvent event );
	void updateMousePosition(MouseEvent event);
	////
	
	// PARAMS
	params::InterfaceGl	mParams;
	
	// CAMERA
	CameraPersp			mCam;
	Quatf				mSceneRotation;
	Vec3f				mEye, mCenter, mUp;
	float				mCameraDistance;
	BoidController		flock_one;
	BoidController		flock_two;
	bool				mSaveFrames;
	bool				mIsRenderingPrint;
	double				changeInterval;
	time_t				lastChange;
	
	
	Capture				capture;
	gl::Texture			texture;
	int					cvThreshholdLevel;
	Matrix44<float>		imageToScreenMap;
	
private:
	SilhouetteDetector	*silhouetteDetector;
	vector<Vec2i_ptr_vec> * polygons;
};

void edgeDetectArea(Surface *surface, Area area);

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
	changeInterval		= 10.0;
	time(&lastChange);
	
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
		silhouetteDetector = new SilhouetteDetector(320,240);
	} catch ( ... ) {
		console() << "Failed to initialize capture device" << std::endl;
	}
	cvThreshholdLevel = 45;		
	// CREATE PARTICLE CONTROLLER
	flock_one.addBoids( NUM_INITIAL_PARTICLES );
	flock_two.addBoids( NUM_INITIAL_PARTICLES );
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
	mParams.addParam( "CV Threshhold", &silhouetteDetector->cvThresholdLevel, "min=0 max=255 step=1 keyIncr=t keyDecr=T" );
	
	//setup transformation from camera space to opengl world space
	imageToScreenMap.setToIdentity();
	imageToScreenMap.translate(Vec3f(getWindowSize().x/2, getWindowSize().y/2, 0));	//translate over and down
	imageToScreenMap.scale(Vec3f(-1*getWindowSize().x/320.0f, -1*getWindowSize().y/240.0f,1.0f));	//scale up
	polygons = new vector<Vec2i_ptr_vec>();
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
	mEye	= Vec3f( 0.0f, 0.0f, mCameraDistance );
	mCam.lookAt( mEye, mCenter, mUp );
	gl::setMatrices( mCam );
	gl::rotate( mSceneRotation);
	
	// CUBE CODE
	glEnable( GL_TEXTURE_2D );
	gl::enableDepthRead();
	gl::enableDepthWrite();	
	//
	
	
	//if (checkTime()) {
	//		flock_one.flatten = !flock_one.flatten;
	//		flock_two.flatten = !flock_two.flatten;
	//	}
	
	
	//OpenCV IO
	//Only do OpenCV business if capture device is open and a new frame is ready
	if( capture && capture.checkNewFrame() ) {		
		polygons->clear();
		ci::Surface captureSurface = capture.getSurface();
		ci::Surface outputSurface = captureSurface;
		silhouetteDetector->processSurface(&captureSurface,polygons,&outputSurface);	//this only works because processSurface doesn't retain either pointer
		
		texture = outputSurface;
		flock_one.applySilhouetteToBoids(polygons,&imageToScreenMap);
		flock_two.applySilhouetteToBoids(polygons,&imageToScreenMap);
	}	 
	
	flock_one.applyForceToBoids();
	if( flock_one.centralGravity ) flock_one.pullToCenter( mCenter );
	flock_one.update();
	
	flock_two.applyForceToBoids();
	if( flock_two.centralGravity) flock_two.pullToCenter( mCenter);
	flock_two.update();
	
}

// Mouse Code ///

void BoidsApp::mouseDown( MouseEvent event )
{
	flock_one.mMousePressed = true;
	flock_two.mMousePressed = true;
	
	BoidsApp::updateMousePosition(event);
}

void BoidsApp::mouseUp( MouseEvent event )
{
	flock_one.mMousePressed = false;
	flock_two.mMousePressed = false;
}

void BoidsApp::mouseDrag( MouseEvent event )
{
	updateMousePosition(event);
}

//NOTE: The mouse position is based on a camera viewing from the initialized orientation only. If you rotate the view of the world, the mapping fails...
void BoidsApp::updateMousePosition(MouseEvent event){
	//flock_one.mousePos = Vec3f(-1*((event.getPos().x)-(getWindowSize().x/2)), -1*((getWindowSize().y/2)-(event.getPos().y)), 0.0f);
//	flock_two.mousePos = Vec3f(-1*((event.getPos().x)-(getWindowSize().x/2)), -1*((getWindowSize().y/2)-(event.getPos().y)), 0.0f);
	flock_one.mousePos = Vec3f(((event.getPos().x)-(getWindowSize().x/2)), ((getWindowSize().y/2)-(event.getPos().y)), 0.0f);
	flock_two.mousePos = Vec3f(((event.getPos().x)-(getWindowSize().x/2)), ((getWindowSize().y/2)-(event.getPos().y)), 0.0f);
}

// END MOUSE CODE//

void BoidsApp::draw()
{	
	gl::clear( Color( 0, 0, 0 ), true );
	gl::enableDepthRead();
	gl::enableDepthWrite();
	
	glColor3f(1.0f,0.0f,0.0f);
	glBegin(GL_POINTS);
	glVertex3f(10.0f,0.0f,1.0f);
	glColor3f(0.0f,1.0f,0.0f);
	glVertex3f(0.0f,10.0f,1.0f);
	glEnd();
	flock_one.draw();
	flock_two.draw();
	drawCapture();
	
	//DRAW THE POLYGONS
	//glPushMatrix();
	gl::pushModelView();
	gl::multModelView(imageToScreenMap);
	glTranslatef(0.0f,0.0f,0.1f);
	//glTranslatef(-1*getWindowSize().x/2,  -1*getWindowSize().y/2, 0.1f);
	//glScalef(getWindowSize().x/320.0f, getWindowSize().y/240.0f,1.0);	//scale up to fill the screen, same as for the video image
	
	glColor3f(1.0f,0.0f,0.0f);
	glLineWidth(5.0f);
	for(vector<Vec2i_ptr_vec>::iterator polygon = polygons->begin(); polygon!=polygons->end();++polygon) {
		glBegin(GL_LINE_STRIP);
		for(vector<Vec2i_ptr>::iterator point = polygon->get()->begin(); point!=polygon->get()->end();++point) {
			glVertex2f(point->get()->x,point->get()->y);
		}
		glEnd();
	}
	gl::popModelView();
	
	if( mSaveFrames ){
		writeImage( getHomeDirectory() + "flocking/image_" + toString( getElapsedFrames() ) + ".png", copyWindowSurface() );
	}
	
	
	// DRAW PARAMS WINDOW
	params::InterfaceGl::draw();
}

void BoidsApp::drawCapture(){
	if( texture){
		gl::color( ColorA( 1.0f, 1.0f, 1.0f, 1.0f ) );
		//texture.bind();
		gl::pushModelView();
		gl::multModelView(imageToScreenMap);
		gl::draw(texture);
		gl::popModelView();
	}
}


bool BoidsApp::checkTime()
{
	time_t newTime = time(&newTime);
	double dif;
	
	dif = difftime(newTime,lastChange);
	
	if ( dif >= changeInterval) {
		lastChange = newTime;
		return TRUE;
	} else {
		return FALSE;
	}
}

CINDER_APP_BASIC( BoidsApp, RendererGl )
