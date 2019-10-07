#include "ofApp.h"

// for

//--------------------------------------------------------------
void ofApp::setup(){
    // init realsense
    realSense = RSDevice::createUniquePtr();
    realSense->checkConnectedDialog();
    // depth frame size can be:
    // 1280x720, 848X480, 640x480, 640x360, 480x270, or 424x240
    int w = 640;
    int h = 360;
    realSense->setDepthSize(w, h);
    realSense->setVideoSize(w, h);
    realSense->capture();
    realSense->printDeviceInfo();
    realSense->setBlackWhiteDepth();
    realSense->enableAlignment(); //aligns depth image with RGB image, will slow framerate down slightly
    
    // opencv images init
    colorImg.allocate(w, h);
    grayImage.allocate(w, h);
    grayThreshNear.allocate(w, h);
    grayThreshFar.allocate(w, h);
    nearThreshold = 255;
    farThreshold = 200;
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
    // point cloud
    bDrawPointCloud = false;
    easyCam.setNearClip(0.01);
    easyCam.setDistance(1);
    
    // contour finder
    contourFinder.setMinAreaRadius(40);
    contourFinder.setMaxAreaRadius(500);
    contourFinder.setFindHoles(false);
}

//--------------------------------------------------------------
void ofApp::update(){
    ofBackground(100, 100, 100);
    // update and if there is a new frame
    if (realSense->update() && !bDrawPointCloud) {
        // load grayscale depth image from the realsense source
        ofPixels pix;
        pix = realSense->getDepthFrame();
        pix.setImageType(OF_IMAGE_GRAYSCALE);
        grayImage.setFromPixels(pix);
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            // or we do it ourselves - show people how they can work with the pixels
            ofPixels & pix = grayImage.getPixels();
            int numPixels = pix.size();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        
        // find contours
        contourFinder.findContours(grayImage);
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255, 255, 255);
    if(bDrawPointCloud && realSense->mDepthFrame.isAllocated()) {
        easyCam.begin();
        drawPointCloud();
        easyCam.end();
    } else {
        realSense->drawDepthStream(ofRectangle(10, 10, 400 , 300));
        realSense->drawVideoStream(ofRectangle(420, 10, 400 , 300));
        grayImage.draw(10, 320, 400, 300);
        
        //draw contours
        ofPushMatrix();
        ofTranslate(10, 320);
        ofScale(400.f/640.f, 300.f/360.f);
        ofPushStyle();
        ofSetColor(255,0,0);
        contourFinder.draw();
        ofPopStyle();
        ofPopMatrix();
    }
    
    // draw instructions
    ofSetColor(255, 255, 255);
    stringstream reportStream;
    
    reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
    << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
    << "set near threshold " << nearThreshold << " (press: + -)" << endl
    << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.size()
    << ", fps: " << ofGetFrameRate() << endl;
    
    ofDrawBitmapString(reportStream.str(), 20, 652);
}

//--------------------------------------------------------------
void ofApp::drawPointCloud() {
    int w = realSense->getDepthWidth();
    int h = realSense->getDepthHeight();
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    rs2::depth_frame df = realSense->rs2Depth;
    const uint16_t* depth_frame = reinterpret_cast<const uint16_t*>(df.get_data());

    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(depth_frame[x+y*realSense->getDepthWidth()] > 0) {
                mesh.addColor(realSense->getVideoFrame().getColor(x,y));
                mesh.addVertex(realSense->getSpacePointFromDepthFrameCoord(glm::vec2(x,y)));
            }
        }
    }

    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        case ' ':
            bThreshWithOpenCV = !bThreshWithOpenCV;
            break;
            
        case'p':
            easyCam.setNearClip(0.01);
            easyCam.setDistance(1);
            bDrawPointCloud = !bDrawPointCloud;
            break;
            
        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

void ofApp::exit(){
    realSense->stop();
}
