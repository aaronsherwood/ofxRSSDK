#include <exception>
#include "ofxRSSDKv2.h"
#include "ofMain.h"

//sdjk https://github.com/IntelRealSense/librealsense
//extrinsic data: https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/examples/sensor-control/api_how_to.h#L209
// projection: https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0
// howtos: https://github.com/IntelRealSense/librealsense/wiki/API-How-To#get-depth-units
// Aligning: https://github.com/IntelRealSense/librealsense/tree/master/examples/align
// D400 data sheet: https://www.mouser.com/pdfdocs/Intel_D400_Series_Datasheet.pdf

/*
Z [16 bits] Depth Only Mode
1280x720 6,15,30
848X480 6,15,30,60,90
640x480 6,15,30,60,90
640x360 6,15,30,60,90
480x270 6,15,30,60,90
424x240 6,15,30,60,90
*/

namespace ofxRSSDK
{	

	RSDevice::~RSDevice(){
	}

	RSDevice::RSDevice()
		:rs2Filter_DispIn(true)
		,rs2Filter_DispOut(false)

	{
		mIsInit = false;
		mIsRunning = false;
		isUsingFilterDec = false;
		isUsingFilterSpat = false;
		isUsingFilterTemp = false;
		isUsingFilterDisparity = false;
		isUsingPostProcessing = false;

		mIsRunning = false;
		mIsCapturing = false;
		mIsPaused = false;
		mIsPlayback = false;
		mIsRecording = false;
        mAlign = false;

		depthWidth = 848;
		depthHeight = 480;
		videoWidth = 1280;
		videoHeight = 720;

		// setting up ofParameters
		param_usePostProcessing.set("use PostProcessing", false);
		param_usePostProcessing.addListener(this, &RSDevice::usePostProcessing_p);

		param_filterDecimation.set("use decimation filter", false);
		param_filterDecimation.addListener(this, &RSDevice::filterDecimation_p);

		param_filterDecimation_mag.set("decimation magnitude", 2, 2, 8);
		param_filterDecimation_mag.addListener(this, &RSDevice::filterDecimation_mag_p);

		param_filterSpatial.set("use spatial filter", false);
		param_filterSpatial.addListener(this, &RSDevice::filterSpatial_p);

		param_filterSpatial_mag.set("spatial magnitude", 2, 2, 5);
		param_filterSpatial_mag.addListener(this, &RSDevice::filterSpatial_mag_p);

		param_filterSpatial_smoothAlpha.set("spatial smoothAlpha", 0.5, 0.25, 1.0);
		param_filterSpatial_smoothAlpha.addListener(this, &RSDevice::filterSpatial_smoothAlpha_p);

		param_filterSpatial_smoothDelta.set("spatial smoothDelta", 20, 1, 50);
		param_filterSpatial_smoothDelta.addListener(this, &RSDevice::filterSpatial_smoothDelta_p);

		param_filterTemporal.set("use temporal Filter", false);
		param_filterTemporal.addListener(this, &RSDevice::filterTemporal_p);

		param_filterTemporal_smoothAlpha.set("temporal smoothAlpha", 0.4, 0.0, 1.0);
		param_filterTemporal_smoothAlpha.addListener(this, &RSDevice::filterTemporal_smoothAlpha_p);

		param_filterTemporal_smoothDelta.set("temporal smoothDelta", 20, 0, 100);
		param_filterTemporal_smoothDelta.addListener(this, &RSDevice::filterTemporal_smoothDelta_p);

		param_filterTemporal_persistency.set("temporal persistency", 3, 0, 8);
		param_filterTemporal_persistency.addListener(this, &RSDevice::filterTemporal_persistency_p);

		param_filterDisparities.set("use disparity filters", false);
		param_filterDisparities.addListener(this, &RSDevice::filterDisparities_p);

		param_deviceLaser.set("use laser projector", true);
		param_deviceLaser.addListener(this, &RSDevice::deviceLaser_p);

		param_deviceLaser_mag.set("laser power", 0.5, 0., 1.0);
		param_deviceLaser_mag.addListener(this, &RSDevice::deviceLaser_mag_p);

		param_deviceAutoExposure.set("use auto exposure", true);
		param_deviceAutoExposure.addListener(this, &RSDevice::deviceAutoExposure_p);

		param_deviceExposure_mag.set("manual exposure", 8500, 20, 100000);
		param_deviceExposure_mag.addListener(this, &RSDevice::deviceExposure_mag_p);

		param_deviceGain_mag.set("image gain", 16, 16, 248);
		param_deviceGain_mag.addListener(this, &RSDevice::deviceGain_mag_p);

		param_deviceFrameQueSize_mag.set("frame queue size", 16, 0, 32);
		param_deviceFrameQueSize_mag.addListener(this, &RSDevice::deviceFrameQueSize_mag_p);

		param_deviceProjectorTemparature.set("projector temp.", "");
		param_deviceAsicTemparature.set("device temp.", "");

		param_recordingPath.set("recordingPath", ofFilePath::getCurrentExeDir() + "data/pointRecording.bag");

		// create new pipeline
		rs2Pipe = std::make_shared<rs2::pipeline>();
	}

#pragma region Init

#pragma endregion
    
    void RSDevice::setBlackWhiteDepth(){
        rs2Color_map.set_option(RS2_OPTION_VISUAL_PRESET, 1.f);
        rs2Color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f); // White to Black
    }

	bool RSDevice::capture()
	{
		return start(CaptureMode::Capture);
	}

	bool RSDevice::record()
	{
		return start(CaptureMode::Recording);
	}

	bool RSDevice::playback()
	{
		return start(CaptureMode::Playback);
	}

	bool RSDevice::start(int _captureMode)
	{
		if (countDevicesAttached()) {
			return start(_captureMode, how_to::get_device_serial(how_to::get_device(0)));
		}
		else {
			ofLogError("Cannot start device. No devices attaches.");
		}
		return false;
	}

	bool RSDevice::start(int _captureMode, const std::string &serial)
	{
		if (mIsRunning) { // if the device has already been started, we stop it before restarting again
			stop();
		}

		switch (_captureMode){
		case CaptureMode::Capture: 
			mIsCapturing = true;
			break;
		case CaptureMode::Playback:
			mIsPlayback = true;
			break;
		case CaptureMode::Recording:
			mIsRecording = true;
			break;
		}
		if (countDevicesAttached()) {

			mVideoStreamSize.x = videoWidth;
			mVideoStreamSize.y = videoHeight;

			mInfraredStreamSize.x = depthWidth;
			mInfraredStreamSize.y = depthHeight;

			//Create a configuration for configuring the pipeline with a non default profile
			rs2::config cfg;

			//Add desired streams to configuration
			//cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16); // Enable default depth
												 // For the color stream, set format to RGBA
												 // To allow blending of the color frame on top of the depth frame
			//cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);

			cfg.enable_device(serial);
			cfg.enable_stream(RS2_STREAM_DEPTH, depthWidth, depthHeight, RS2_FORMAT_Z16, 30);
			cfg.enable_stream(RS2_STREAM_COLOR, videoWidth, videoHeight, RS2_FORMAT_RGB8, 30);
			cfg.enable_stream(RS2_STREAM_INFRARED, 1);

			if (mIsPlayback) {
				cfg.enable_device_from_file(param_recordingPath.get());
			}else if (mIsRecording) {
				cfg.enable_record_to_file(param_recordingPath.get());
			}
				
			rs2PipeLineProfile = rs2Pipe->start(cfg);
            
            align_to = find_stream_to_align(rs2PipeLineProfile.get_streams());

			rs2Device = rs2PipeLineProfile.get_device();
            
            if (mIsCapturing) {
                // Capture 30 frames to give autoexposure, etc. a chance to settle
                for (auto i = 0; i < 30; ++i) rs2Pipe->wait_for_frames();
            }

			auto depth_stream = rs2PipeLineProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
			rs2DepthIntrinsics = depth_stream.get_intrinsics();

			auto video_stream = rs2PipeLineProfile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
			rs2VideoIntrinsics = video_stream.get_intrinsics();

			auto infra_stream = rs2PipeLineProfile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
			rsInfraLeftIntrinsics = infra_stream.get_intrinsics();

			


			mIsRunning = true;

			return true;

		}
		else {
			ofLogError("Cannot start device. No devices attaches.");
		}

		return false;
	}
    
    void RSDevice::enableAlignment(){
        mAlign = true;
    }
    
    rs2_stream RSDevice::find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
    {
        //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
        //We prioritize color streams to make the view look better.
        //If color is not available, we take another stream that (other than depth)
        rs2_stream align_to = RS2_STREAM_ANY;
        bool depth_stream_found = false;
        bool color_stream_found = false;
        for (rs2::stream_profile sp : streams)
        {
            rs2_stream profile_stream = sp.stream_type();
            if (profile_stream != RS2_STREAM_DEPTH)
            {
                if (!color_stream_found)         //Prefer color
                    align_to = profile_stream;
                
                if (profile_stream == RS2_STREAM_COLOR)
                {
                    color_stream_found = true;
                }
            }
            else
            {
                depth_stream_found = true;
            }
        }
        
        if(!depth_stream_found)
            throw std::runtime_error("No Depth stream available");
        
        if (align_to == RS2_STREAM_ANY)
            throw std::runtime_error("No stream found to align with Depth");
        
        return align_to;
    }
    
    bool RSDevice::profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
    {
        for (auto&& sp : prev)
        {
            //If previous profile is in current (maybe just added another)
            auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
            if (itr == std::end(current)) //If it previous stream wasn't found in current
            {
                return true;
            }
        }
        return false;
    }

	bool RSDevice::stop()
	{
		rs2Pipe->stop();
		// create new pipeline for next purpose
		rs2Pipe = std::make_shared<rs2::pipeline>();

		mIsRunning = false;
		mIsCapturing = false;
		mIsPaused = false;
		mIsPlayback = false;
		mIsRecording = false;
		return true;
	}

	bool RSDevice::isRunning() {
		return mIsRunning;
	}

	bool RSDevice::update()
	{
		// polling for the next frame
		if (rs2Pipe->poll_for_frames(&rs2FrameSet))
		{
            //safe guard fo use during alignment
            //set false each time not safe to proceed
            bool proceed=true;
            
            // if there is a frame...
			// get the depth data from the frame
            
            // and align if told to do so
            if (mAlign){
                if (profile_changed(rs2Pipe->get_active_profile().get_streams(), rs2PipeLineProfile.get_streams())){
                    align_to = find_stream_to_align(rs2PipeLineProfile.get_streams());
                }
                rs2::align align(align_to);
                auto processed = align.process(rs2FrameSet);
                if (processed.size()==2){
                    rs2::video_frame other_frame = processed.first(align_to);
                    // aligned depth image
                    rs2Depth = processed.get_depth_frame();
                } else {
                    proceed=false;
                }
            } else{
                rs2Depth = rs2FrameSet.first(RS2_STREAM_DEPTH);
            }
            

            /* Apply filters.
            The implemented flow of the filters pipeline is in the following order:
            1. apply decimation filter
            2. apply threshold filter
            3. transform the scene into disparity domain
                4. apply spatial filter
                5. apply temporal filter
            6. revert the results back (if step Disparity filter was applied
                to depth domain (each post processing block is optional and can be applied independantly).
            */
            if (proceed){
                // apply postprocessing filters on the depth data
                if (isUsingPostProcessing) {
                    if (isUsingFilterDec) {
                        rs2Depth = rs2Filter_dec.process(rs2Depth);
                    }
                    if (isUsingFilterDisparity) {
                        rs2Depth = rs2Filter_DispIn.process(rs2Depth);
                    }
                    if (isUsingFilterSpat) {
                        rs2Depth = rs2Filter_spat.process(rs2Depth);
                    }
                    if (isUsingFilterTemp) {
                        rs2Depth = rs2Filter_temp.process(rs2Depth);
                    }
                    if (isUsingFilterDisparity) {
                        rs2Depth = rs2Filter_DispOut.process(rs2Depth);
                    }
                }

                // Use the colorizer to get an rgb image for the depth stream
                auto rs2DepthVideoFrame = rs2Color_map.colorize(rs2Depth);

                // set the new resolutions for the depth stream
                mDepthStreamSize.x = rs2DepthVideoFrame.get_width();
                mDepthStreamSize.y = rs2DepthVideoFrame.get_height();

                // populate the depth video frame
                mDepthFrame.setFromExternalPixels((unsigned char *)rs2DepthVideoFrame.get_data(), mDepthStreamSize.x, mDepthStreamSize.y, 3);

                // get the video stream from the camera frame
                auto rs2VideoFrame = rs2FrameSet.first(RS2_STREAM_COLOR).as<rs2::video_frame>();
                // populate the video frame
                mVideoFrame.setFromExternalPixels((unsigned char *)rs2VideoFrame.get_data(), rs2VideoFrame.get_width(), rs2VideoFrame.get_height(), 3);

                // get the infrared stream from the camera frame
                auto rs2IRFrame = rs2FrameSet.first(RS2_STREAM_INFRARED).as<rs2::video_frame>();
                // populate the infrared frame
                mInfraLeftFrame.setFromExternalPixels((unsigned char *)rs2IRFrame.get_data(), rs2IRFrame.get_width(), rs2IRFrame.get_height(), 1);
            }
			param_deviceProjectorTemparature = ofToString(get_deviceProjectorTemperature()) +  " [deg]";
			param_deviceAsicTemparature = ofToString(get_deviceProjectorTemperature()) + " [deg]";

			return true;
		}
		return false;
	}


#pragma region Enable

#pragma endregion

#pragma region Update


	bool RSDevice::drawVideoStream(const ofRectangle & rect)
	{
		if (mVideoFrame.getWidth() > 0) {
			ofTexture texRGB;
			texRGB.loadData(mVideoFrame);
			texRGB.draw(rect.x, rect.y, rect.width, rect.height);
			return true;
		}
		return false;
	}

	bool RSDevice::drawDepthStream(const ofRectangle & rect)
	{
		if (mDepthFrame.getWidth() > 0) {
			ofTexture texRGB;
			texRGB.loadData(mDepthFrame);
			texRGB.draw(rect.x, rect.y, rect.width, rect.height);
			return true;
		}
		return false;
	}

	bool RSDevice::drawInfraLeftStream(const ofRectangle & rect)
	{
		if (mInfraLeftFrame.getWidth() > 0) {
			ofTexture texRGB;
			texRGB.loadData(mInfraLeftFrame);
			texRGB.draw(rect.x, rect.y, rect.width, rect.height);
			return true;
		}
		return false;
	}

#pragma endregion

#pragma region Getters

	int RSDevice::get_deviceAsicTemperature()
	{
		auto rs2DepthSensor = rs2Device.first<rs2::depth_sensor>();

		if (rs2DepthSensor.supports(RS2_OPTION_ASIC_TEMPERATURE))
		{
			return (int)rs2DepthSensor.get_option(RS2_OPTION_ASIC_TEMPERATURE);
		}
		return 0;
	}

	int RSDevice::get_deviceProjectorTemperature()
	{
		auto rs2DepthSensor = rs2Device.first<rs2::depth_sensor>();

		if (rs2DepthSensor.supports(RS2_OPTION_PROJECTOR_TEMPERATURE))
		{
			return (int)rs2DepthSensor.get_option(RS2_OPTION_PROJECTOR_TEMPERATURE);
		}
		return 0;
	}

	const ofPixels& RSDevice::getVideoFrame()
	{
		return mVideoFrame;
	}

	const ofPixels& RSDevice::getInfraLeftFrame()
	{
		return mInfraLeftFrame;
	}

	const ofPixels& RSDevice::getDepthFrame()
	{
		return mDepthFrame;
	}

	//Nomenclature Notes:
//	"Space" denotes a 3d coordinate
//	"Image" denotes an image space point ((0, width), (0,height), (image depth))
//	"Coords" denotes texture space (U,V) coordinates
//  "Frame" denotes a full Surface

	glm::vec3 RSDevice::getSpacePointFromDepthFrameCoord(glm::vec2 depthCoordinate) {
		rs2::depth_frame dFrame = rs2Depth.as<rs2::depth_frame>();
	
		float d_pt[3] = { 0 };
		float d_px[2] = { depthCoordinate.x, depthCoordinate.y };

		float depth = dFrame.get_distance(d_px[0], d_px[1]);

		rs2_deproject_pixel_to_point(d_pt, &rs2DepthIntrinsics, d_px, depth);

		return glm::vec3(d_pt[0], d_pt[1], d_pt[2]);
	}

	glm::vec3 RSDevice::getSpacePointFromVideoFrameCoord(glm::vec2 videoCoordinate) {
		return getSpacePointFromDepthFrameCoord(glm::vec2(videoCoordinate.x / mVideoStreamSize.x * mDepthStreamSize.x, videoCoordinate.y / mVideoStreamSize.y * mDepthStreamSize.y));
	}

	glm::vec3 RSDevice::getSpacePointFromInfraLeftFrameCoord(glm::vec2 infraCoordinate) {
		return getSpacePointFromDepthFrameCoord(glm::vec2(infraCoordinate.x / mInfraredStreamSize.x * mDepthStreamSize.x, infraCoordinate.y / mInfraredStreamSize.y * mDepthStreamSize.y));
	}

	float RSDevice::getSpaceDistanceFromDepthFrame(glm::vec2 depthCoordinate) {
		rs2::depth_frame depthFrame = rs2Depth.as<rs2::depth_frame>();
		return depthFrame.get_distance(depthCoordinate.x, depthCoordinate.y);
	}

	void RSDevice::setVideoSize(int width, int height)
	{
		videoWidth = width;
		videoHeight = height;
	}

	void RSDevice::setDepthSize(int width, int height)
	{
		depthWidth = width;
		depthHeight = height;
	}

	void RSDevice::checkConnectedDialog() {
		rs2::context ctx;
		auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
		while (list.size() == 0) {
			ofSystemAlertDialog("No Device found. Have you attached a RealSense D400 camera?");
			list = ctx.query_devices();
		}
	}

	void RSDevice::usePostProcessing_p(bool & enable) {
		usePostProcessing(enable);
	}
	void RSDevice::usePostProcessing(bool const & enable) {
		isUsingPostProcessing = enable;
	}

	void RSDevice::filterDecimation_p(bool & enable) {
		filterDecimation(enable);
	}
	void RSDevice::filterDecimation(bool const & enable) {
		isUsingFilterDec = enable;
	}

	void RSDevice::filterDecimation_mag_p(int & magnitude) {
		filterDecimation_mag(magnitude);
	}
	void RSDevice::filterDecimation_mag(int const & magnitude) {
		rs2Filter_dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, magnitude);
	}

	void RSDevice::filterSpatial_p(bool & enable) {
		filterSpatial(enable);
	}
	void RSDevice::filterSpatial(bool const & enable) {
		isUsingFilterSpat = enable;
	}

	void RSDevice::filterSpatial_mag_p(int & magnitude) {
		filterSpatial_mag(magnitude);
	}
	void RSDevice::filterSpatial_mag(int const & magnitude) {
		rs2Filter_spat.set_option(RS2_OPTION_FILTER_MAGNITUDE, magnitude);
	}

	void RSDevice::filterSpatial_smoothAlpha_p(float & smoothAlpha) {
		filterSpatial_smoothAlpha(smoothAlpha);
	}
	void RSDevice::filterSpatial_smoothAlpha(float const & smoothAlpha) {
		rs2Filter_temp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, smoothAlpha);
	}

	void RSDevice::filterSpatial_smoothDelta_p(int & smoothDelta) {
		filterSpatial_smoothDelta(smoothDelta);
	}
	void RSDevice::filterSpatial_smoothDelta(int const & smoothDelta) {
		rs2Filter_spat.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, smoothDelta);
	}

	void RSDevice::filterSpatial_holeFilling_p(int & holeFilling) {
		filterSpatial_holeFilling(holeFilling);
	}
	void RSDevice::filterSpatial_holeFilling(int const & holeFilling) {
		rs2Filter_spat.set_option(RS2_OPTION_HOLES_FILL, holeFilling);
	}

	void RSDevice::filterTemporal_p(bool & enable) {
		filterTemporal(enable);
	}
	void RSDevice::filterTemporal(bool const & enable) {
		isUsingFilterTemp = enable;
	}

	void RSDevice::filterTemporal_smoothAlpha_p(float & smoothAlpha) {
		filterTemporal_smoothAlpha(smoothAlpha);
	}
	void RSDevice::filterTemporal_smoothAlpha(float const & smoothAlpha) {
		rs2Filter_temp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, smoothAlpha);
	}

	void RSDevice::filterTemporal_smoothDelta_p(int & smoothDelta) {
		filterTemporal_smoothDelta(smoothDelta);
	}
	void RSDevice::filterTemporal_smoothDelta(int const & smoothDelta) {
		rs2Filter_temp.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, smoothDelta);
	}

	void RSDevice::filterTemporal_persistency_p(int & persitency) {
		filterTemporal_persistency(persitency);
	}
	void RSDevice::filterTemporal_persistency(int const & persitency) {
		rs2Filter_temp.set_option(RS2_OPTION_HOLES_FILL, persitency);
	}

	void RSDevice::filterDisparities_p(bool & enable) {
		filterDisparities(enable);
	}
	void RSDevice::filterDisparities(bool const & enable) {
		isUsingFilterDisparity = enable;
	}

	/////////////////////////////////////////////////////////////
	//
	// DEVICE SETTINGS
	//
	/////////////////////////////////////////////////////////////

	// LASER

	void RSDevice::deviceLaser_mag(float const & magnitude) {
		auto rs2DepthSensor = rs2Device.first<rs2::depth_sensor>();

		if (rs2DepthSensor.supports(RS2_OPTION_EMITTER_ENABLED))
		{
			if (rs2DepthSensor.supports(RS2_OPTION_LASER_POWER))
			{
				// Query min and max values:
                
				auto range = rs2DepthSensor.get_option_range(RS2_OPTION_LASER_POWER);
                float amount = range.max * magnitude;
                cout<<range.min<<" "<<range.max<<" "<<magnitude<<" "<<amount<<endl;
				rs2DepthSensor.set_option(RS2_OPTION_LASER_POWER, amount); // Set max power
			}
		}
	}
	void RSDevice::deviceLaser_mag_p(float & magnitude) {
		deviceLaser_mag(magnitude);
	}

	void RSDevice::deviceLaser(bool const & enable) {
		auto rs2DepthSensor = rs2Device.first<rs2::depth_sensor>();

		if (rs2DepthSensor.supports(RS2_OPTION_EMITTER_ENABLED))
		{
			rs2DepthSensor.set_option(RS2_OPTION_EMITTER_ENABLED, (enable)?1.:0.); // Enable emitter
		}
	}
	void RSDevice::deviceLaser_p(bool & enable) {
		deviceLaser(enable);
	}

	// EXPOSURE

	void RSDevice::deviceAutoExposure(bool const & enable) {
		auto rs2DepthSensor = rs2Device.first<rs2::depth_sensor>();

		if (rs2DepthSensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
		{
			rs2DepthSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, (enable) ? 1. : 0.); // Enable emitter
		}
	}

	void RSDevice::deviceAutoExposure_p(bool & enable) {
		deviceAutoExposure(enable);
	}

	void RSDevice::deviceExposure_mag(int const & magnitude) {
		auto rs2DepthSensor = rs2Device.first<rs2::depth_sensor>();

		if (rs2DepthSensor.supports(RS2_OPTION_EXPOSURE))
		{
			// Query min and max values:
			auto range = rs2DepthSensor.get_option_range(RS2_OPTION_EXPOSURE);
			if(range.min <= magnitude && magnitude <= range.max)
				rs2DepthSensor.set_option(RS2_OPTION_EXPOSURE, magnitude);
		}
	}
	void RSDevice::deviceExposure_mag_p(int & magnitude) {
		deviceExposure_mag(magnitude);
	}

	void RSDevice::deviceFrameQueSize_mag(int const & magnitude) {
		auto rs2DepthSensor = rs2Device.first<rs2::depth_sensor>();

		if (rs2DepthSensor.supports(RS2_OPTION_FRAMES_QUEUE_SIZE))
		{
			// Query min and max values:
			auto range = rs2DepthSensor.get_option_range(RS2_OPTION_FRAMES_QUEUE_SIZE);
			if (range.min <= magnitude && magnitude <= range.max)
				rs2DepthSensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, magnitude);
		}
	}
	void RSDevice::deviceFrameQueSize_mag_p(int & magnitude) {
		deviceFrameQueSize_mag(magnitude);
	}

	void RSDevice::deviceGain_mag(int const & magnitude) {
		// we are only setting image gain if auto exposure is off, otherwise we have crashes.
		if (!param_deviceAutoExposure.get()) {
			auto rs2DepthSensor = rs2Device.first<rs2::depth_sensor>();

			if (rs2DepthSensor.supports(RS2_OPTION_GAIN))
			{
				// Query min and max values:
				auto range = rs2DepthSensor.get_option_range(RS2_OPTION_GAIN);
				if (range.min <= magnitude && magnitude <= range.max)
					rs2DepthSensor.set_option(RS2_OPTION_GAIN, magnitude);
			}
		}
	}
	void RSDevice::deviceGain_mag_p(int & magnitude) {
		deviceGain_mag(magnitude);
	}

	int RSDevice::countDevicesAttached()
	{
		return how_to::nof_devices_attached();
	}

	const std::string RSDevice::getSerialNumber(int index)
	{
		if (index == -1) {
			if (isRunning()) {
				return how_to::get_device_serial(rs2Device);
			}
			else {
				return how_to::get_device_serial(how_to::get_device(0));
			}
		}  
		return how_to::get_device_serial(how_to::get_device(index));
	}

	void RSDevice::printDeviceInfo()
	{
		if (isRunning()) {
			how_to::print_device_information(rs2Device);
		}
		else {
			if (how_to::nof_devices_attached() > 0) {
				how_to::print_device_information(how_to::get_device(0));
			}
		}
	}

	float RSDevice::get_depth_scale(rs2::device dev)
	{
		// Go over the device's sensors
		for (rs2::sensor& sensor : dev.query_sensors())
		{
			// Check if the sensor if a depth sensor
			if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
			{
				return dpt.get_depth_scale();
			}
		}
		throw std::runtime_error("Device does not have a depth sensor");
	}

	void RSDevice::hardwareReset()
	{
		how_to::hardware_reset_device();
	}

}
#pragma endregion
