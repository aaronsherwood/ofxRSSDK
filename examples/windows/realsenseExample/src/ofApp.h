#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxRSSDKv2.h"
#include "ofxGui.h"

using namespace ofxRSSDK;

class ofApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();
    
    void drawPointCloud();
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    
    RSDevicePtr	realSense;
    
    ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    ofxCv::ContourFinder contourFinder;
    
    bool bThreshWithOpenCV;
    bool bDrawPointCloud;
    
    int nearThreshold;
    int farThreshold;
    
    int angle;
    
    // used for viewing the point cloud
    ofEasyCam easyCam;
};
