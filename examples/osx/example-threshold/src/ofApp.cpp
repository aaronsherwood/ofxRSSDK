#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetWindowShape(1280, 720);

	realSense = RSDevice::createUniquePtr();

	realSense->checkConnectedDialog();

//	realSense->hardwareReset();

	ofLogNotice("Device detected..");
	gui_post.setup("PostProcessing", "postprocessingSetup", 0, 0); // most of the time you don't need a name but don't forget to call setup
	gui_post.add(realSense->param_usePostProcessing);
	gui_post.add(realSense->param_filterDecimation);
	gui_post.add(realSense->param_filterDecimation_mag);
	gui_post.add(realSense->param_filterDisparities);
	gui_post.add(realSense->param_filterSpatial);
	gui_post.add(realSense->param_filterSpatial_smoothAlpha);
	gui_post.add(realSense->param_filterSpatial_smoothDelta);
	gui_post.add(realSense->param_filterSpatial_mag);
	gui_post.add(realSense->param_filterTemporal);
	gui_post.add(realSense->param_filterTemporal_smoothAlpha);
	gui_post.add(realSense->param_filterTemporal_smoothDelta);
	gui_post.add(realSense->param_filterTemporal_persistency);

	//realSense->enablePointCloud(CloudRes::FULL_RES);
//	realSense->setPointCloudRange(100.0f,1000.0f);

	gui_device.setup("Device", "deviceSettings", 200, 0);

	if (realSense->capture()) {
		// the device settings should be loaded/set after the start()
		gui_device.add(realSense->param_deviceLaser);
		gui_device.add(realSense->param_deviceLaser_mag);
		gui_device.add(realSense->param_deviceAutoExposure);
		gui_device.add(realSense->param_deviceExposure_mag);
		gui_device.add(realSense->param_deviceGain_mag);
		gui_device.add(realSense->param_deviceFrameQueSize_mag);
		gui_device.add(realSense->param_deviceAsicTemparature);
		gui_device.add(realSense->param_deviceProjectorTemparature);
        gui_device.add(clipping_threshold.set("threshold",1,0,5)); //default to clip at a meter
	}
    
	realSense->printDeviceInfo();
}

//--------------------------------------------------------------
void ofApp::update()
{
	if (realSense->update()) {
        remove_background(realSense->mDepthFrame, clipping_threshold);
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	realSense->drawDepthStream(ofRectangle(0., 0, ofGetWidth() , ofGetHeight()));

	ofDrawBitmapString("fps: " + ofToString(ofGetFrameRate()), ofGetWidth() - 200, 10);

	gui_post.draw();
	gui_device.draw();
}


void ofApp::exit()
{
	realSense->stop();
}

//--------------------------------------------------------------
void ofApp::remove_background(ofPixelsRef threshold_frame, float clipping_dist)
{
    #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
   
    rs2::depth_frame df = realSense->rs2Depth;
    const uint16_t* depth_frame = reinterpret_cast<const uint16_t*>(df.get_data());
    float depth_scale = .001; //sets the depth to be in meters
    for (int y=0; y<realSense->getDepthHeight(); y++ ){
        for (int x=0; x<realSense->getDepthWidth(); x++ ){
            float distance = depth_scale * depth_frame[x+y*realSense->getDepthWidth()];
            if (distance>clipping_dist){
                threshold_frame.setColor(x, y, ofColor(0));
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

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
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
