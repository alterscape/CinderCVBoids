/*
 *  BoidsApp.h
 *  Boids
 *
 *  Created by Ryan Spicer on 11/9/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#pragma once

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"


using namespace ci;
using namespace ci::app;
using namespace std;

class BoidsApp : public AppBasic {
public:
	void prepareSettings(Settings * settings);
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();
};