#include "ITMFovisTracker.h"
#include "../Utils/lodepng.h"
#include "../Utils/draw.hpp"

using namespace ITMLib::Engine;

// Ref. examples/tum-rgbd/main.cpp

ITMFovisTracker::ITMFovisTracker(Vector2i imgSize, const ITMIntrinsics *intrinsics)
{
	camParams.width = imgSize.width;
	camParams.height = imgSize.height;
	camParams.fx = intrinsics->projectionParamsSimple.fx;
	camParams.fy = intrinsics->projectionParamsSimple.fy;
	camParams.cx = intrinsics->projectionParamsSimple.px;
	camParams.cy = intrinsics->projectionParamsSimple.py;
	fovisRectification = new fovis::Rectification(camParams);
	
 	fovis::VisualOdometryOptions options = fovis::VisualOdometry::getDefaultOptions();
	//options["target-pixels-per-feature"] = "100";
	options["inlier-max-reprojection-error"] = "2.5";
	//options["clique-inlier-threshold"] = "1.0";
	fovis = new fovis::VisualOdometry(fovisRectification, options);
}

ITMFovisTracker::~ITMFovisTracker(void) 
{ 
	delete fovisRectification;
	delete fovis;
}


void ITMFovisTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	// convert RGB data to grayscale
	int w = view->rgb->noDims.width;
	int h = view->rgb->noDims.height;
    	std::vector<uint8_t> gray_data(w*h);
	const Vector4u *rgbData = view->rgb->GetData(MEMORYDEVICE_CPU);
	for(int i=0; i<w*h; i++)
		gray_data[i] = ((ushort)rgbData[i].r + (ushort)rgbData[i].g + (ushort)rgbData[i].b) / 3;

	// convert depth data to a DepthImage object
	fovis::DepthImage depth_image(camParams, w, h);
	std::vector<float> depth_image_data(w*h);
	const float *depthData = view->depth->GetData(MEMORYDEVICE_CPU);
	for (int i=0; i<w*h; i++)
		depth_image_data[i] = depthData[i] > 0 ? depthData[i] : NAN; // todo frustum_max
	depth_image.setDepthImage(&depth_image_data[0]);

	fovis->processFrame(&gray_data[0], &depth_image);

	fovis::MotionEstimateStatusCode status = fovis->getMotionEstimateStatus();
    switch(status) {
      case fovis::INSUFFICIENT_INLIERS:
        std::cout << "insufficient inliers" << std::endl;
        break;
      case fovis::OPTIMIZATION_FAILURE:
        std::cout << "optimization failed" << std::endl;
        break;
      case fovis::REPROJECTION_ERROR:
        std::cout << "reprojection error too high" << std::endl;
        break;
      case fovis::NO_DATA:
        std::cout << "no data" << std::endl;
        break;
      case fovis::SUCCESS:
      default:
        break;
    }

	Eigen::Isometry3d pose = fovis->getPose();
	Eigen::Vector3d t = pose.translation();
	//Eigen::Quaterniond q(pose.rotation());


      /*DrawImage draw_img(w, h*2);
      DrawColor status_colors[] = {
        DrawColor(255, 255, 0),
        DrawColor(0, 255, 0),
        DrawColor(255, 0, 0),
        DrawColor(255, 0, 255),
        DrawColor(127, 127, 0)
      };

      DrawColor red(255, 0, 0);
      DrawColor green(0, 255, 0);
      DrawColor blue(0, 0, 255);

      memset(&draw_img.data[0], 0, draw_img.data.size());

      const fovis::OdometryFrame* ref_frame = fovis->getReferenceFrame();
      const fovis::OdometryFrame* tgt_frame = fovis->getTargetFrame();

      const fovis::PyramidLevel* ref_pyr_base = ref_frame->getLevel(0);
      const uint8_t* ref_img_data = ref_pyr_base->getGrayscaleImage();
      int ref_img_stride = ref_pyr_base->getGrayscaleImageStride();

      const fovis::PyramidLevel* tgt_pyr_base = tgt_frame->getLevel(0);
      const uint8_t* tgt_img_data = tgt_pyr_base->getGrayscaleImage();
      int tgt_img_stride = tgt_pyr_base->getGrayscaleImageStride();

      int tgt_yoff = ref_pyr_base->getHeight();

      // draw the reference frame on top
      draw_gray_img_rgb(ref_img_data, ref_pyr_base->getWidth(),
          ref_pyr_base->getHeight(), ref_img_stride, 0, 0, &draw_img);

      // draw the target frame on bottom
      draw_gray_img_rgb(tgt_img_data, tgt_pyr_base->getWidth(),
          tgt_pyr_base->getHeight(), tgt_img_stride, 0, tgt_yoff, &draw_img);

      const fovis::MotionEstimator* mestimator = fovis->getMotionEstimator();

      int num_levels = ref_frame->getNumLevels();
      for(int level_index=0; level_index<num_levels; level_index++) {
        // draw reference features
        const fovis::PyramidLevel* ref_level = ref_frame->getLevel(level_index);
        int num_ref_keypoints = ref_level->getNumKeypoints();
        for(int ref_kp_ind=0; ref_kp_ind<num_ref_keypoints; ref_kp_ind++) {
          const fovis::KeypointData* ref_kp = ref_level->getKeypointData(ref_kp_ind);
          int ref_u = (int)round(ref_kp->base_uv.x());
          int ref_v = (int)round(ref_kp->base_uv.y());
          draw_box_rgb(ref_u-1, ref_v-1, ref_u+1, ref_v+1, blue, &draw_img);
        }

        // draw target features
        const fovis::PyramidLevel* tgt_level = tgt_frame->getLevel(level_index);
        int num_tgt_keypoints = tgt_level->getNumKeypoints();
        for(int tgt_kp_ind=0; tgt_kp_ind<num_tgt_keypoints; tgt_kp_ind++) {
          const fovis::KeypointData* tgt_kp = tgt_level->getKeypointData(tgt_kp_ind);
          int tgt_u = (int)round(tgt_kp->base_uv.x());
          int tgt_v = (int)round(tgt_kp->base_uv.y());
          draw_box_rgb(tgt_u-1, tgt_v-1 + tgt_yoff, tgt_u+1, tgt_v+1 + tgt_yoff,
              blue, &draw_img);
        }
      }

      const fovis::FeatureMatch* matches = mestimator->getMatches();
      int num_matches = mestimator->getNumMatches();

      // draw non-inlier matches
      for(int match_index=0; match_index<num_matches; match_index++) {
        const fovis::FeatureMatch& match = matches[match_index];
        if(match.inlier)
          continue;
        const fovis::KeypointData* ref_keypoint = match.ref_keypoint;
        const fovis::KeypointData* tgt_keypoint = match.target_keypoint;

        int ref_u = (int)round(ref_keypoint->base_uv.x());
        int ref_v = (int)round(ref_keypoint->base_uv.y());

        int tgt_u = (int)round(tgt_keypoint->base_uv.x());
        int tgt_v = (int)round(tgt_keypoint->base_uv.y());

        draw_line_rgb(ref_u, ref_v,
            tgt_u, tgt_v + tgt_yoff,
            red, &draw_img);
      }

      // draw inlier matches
      for(int match_index=0; match_index<num_matches; match_index++) {
        const fovis::FeatureMatch& match = matches[match_index];
        if(!match.inlier)
          continue;
        const fovis::KeypointData* ref_keypoint = match.ref_keypoint;
        const fovis::KeypointData* tgt_keypoint = match.target_keypoint;

        int ref_u = (int)round(ref_keypoint->base_uv.x());
        int ref_v = (int)round(ref_keypoint->base_uv.y());

        int tgt_u = (int)round(tgt_keypoint->base_uv.x());
        int tgt_v = (int)round(tgt_keypoint->base_uv.y());

        draw_line_rgb(ref_u, ref_v,
            tgt_u, tgt_v + tgt_yoff,
            green, &draw_img);
      }

      // draw a couple lines indicating the VO status
      draw_box_rgb(0, tgt_yoff - 1, draw_img.width,
          tgt_yoff + 1, status_colors[status], &draw_img);

      // save visualization
      std::string vis_fname = "vis.png";
      lodepng::encode(vis_fname, &draw_img.data[0], draw_img.width,
          draw_img.height, LCT_RGB);*/

	
	Matrix3f R; Vector3f T;
	Eigen::Matrix3d rot = pose.rotation();
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R(i,j) = rot(i,j);
	for (int i = 0; i < 3; i++)
		T.v[i] = -t(i);
	trackingState->pose_d->SetRT(R, R * T);
	trackingState->pose_d->Coerce();
}

