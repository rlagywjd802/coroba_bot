/*******************************************/
//ROS HEADERS
/********************************************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/make_shared.hpp>

#include "point_cloud_stitching_color/pcl_fusion_color.hpp"
#include "point_cloud_utilities/pcl_utilities.hpp"

using namespace std;
using namespace fusion;
using namespace pcl;

/***************************************************/
//Global Variables
/***************************************************/
ros::Publisher pub;
static const std::double_t DEFAULT_MINIMUM_TRANSLATION = 0.1;
const bool USE_ICP = false;
std::double_t distance_moved = 0.0;
const double leaf_size = 0.005;
const int neighbors = 512;
const int max_icp_iterations = 100;
const int max_rounds = 1;
bool fuse_point_cloud = true;
bool continuous_mode = true;
bool downsample = false;

class MyPointRepresentation : public pcl::PointRepresentation <PointNormal>
{
  using pcl::PointRepresentation<PointNormal>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

void cleanPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned_cloud)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_voxelized;
    //voxelize the combined cloud so we have some semi clean data that isn't insanely large

    std::cout<<"Voxelizing the points..."<<std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(cloud_in);
    voxelSampler.setLeafSize(leaf_size,leaf_size,leaf_size);
    voxelSampler.filter(cloud_voxelized);

    std::cout<<"MLS..."<<std::endl;

    // ROS_INFO("Performing MLS on the multisampled clouds...");
    // resample the data so we get normals that are reasonable using MLS
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // next perform MLS to average out the Z values and give us something reasonable looking
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud_voxelized.makeShared());
    mls.setPolynomialOrder(3);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.01*10.0); // made this 1 order of magnitude bigger than the voxel size
    mls.setSqrGaussParam(mls.getSearchRadius()*mls.getSearchRadius()); // usually set this to be square of the search radius
    mls.process(mls_points);

    pcl::PointCloud<pcl::PointNormal> filtered_pcl;  
    //do a filter to get rid of any noisey points.
    // ROS_INFO("Removing statistical outliers...");
    // remove statistical outliers

    std::cout<<"Statistical Filtering..."<<std::endl;

    pcl::StatisticalOutlierRemoval<pcl::PointNormal> statFilter;
    statFilter.setInputCloud(mls_points.makeShared());
    statFilter.setMeanK(10);
    statFilter.setStddevMulThresh(0.2);
    // copy the filtered data into cloudfiltered
    statFilter.filter(filtered_pcl);

    std::cout<<"Reached Here..."<<std::endl;

    for(auto point:filtered_pcl.points)
    {
      PointXYZ pt={point.x,point.y,point.z};
      cleaned_cloud->points.push_back(pt);
    }

    std::cout<<"End of Function.."<<std::endl;

  }

PclFusion::PclFusion(ros::NodeHandle& nh,const std::string& fusion_frame,vector<double>& box)//The queue size is kept to one.
  : robot_tform_listener_(tf_buffer_)
  , fusion_frame_T_camera_prev_(Eigen::Affine3d::Identity())
  , fusion_frame_(fusion_frame)
  , bounding_box(box)
{  // Subscribe to point cloud
    point_cloud_sub_ = nh.subscribe("input_point_cloud", 1, &PclFusion::onReceivedPointCloud,this);
    save_point_cloud = nh.advertiseService("/pcl_fusion_node/save_point_cloud", &PclFusion::savePointCloud, this);
    reset_fusion_service_ = nh.advertiseService("/pcl_fusion_node/reset",&PclFusion::resetFusion, this);
    capture_point_cloud = nh.advertiseService("/pcl_fusion_node/capture_point_cloud",&PclFusion::capturePointCloud,this);

    publish_cloud = nh.advertise<sensor_msgs::PointCloud2> ("pcl_fusion_node/fused_points", 1);
}

void PclFusion::onReceivedPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
    // Convert to useful point cloud format
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_in, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud = PCLUtilities::pointCloud2ToPclXYZRGB(pcl_pc2);

    Eigen::Affine3d fusion_frame_T_camera = Eigen::Affine3d::Identity();
    pcl::PointCloud<pcl::PointXYZRGB> cloud_transformed;
    try
    {
        geometry_msgs::TransformStamped transform_fusion_frame_T_camera = tf_buffer_.lookupTransform(fusion_frame_, cloud_in->header.frame_id, ros::Time(0));
        fusion_frame_T_camera = tf2::transformToEigen(transform_fusion_frame_T_camera); 
    }
    catch (tf2::TransformException& ex)
    {
        // Abort integration if tf lookup failed
        ROS_WARN("%s", ex.what());
        return;
    }
    pcl::PointCloud<pcl::PointXYZRGB> temp;
    std::double_t motion_mag = (fusion_frame_T_camera.inverse() * fusion_frame_T_camera_prev_).translation().norm();//Get the distance traversed by the camera.
    if (motion_mag > DEFAULT_MINIMUM_TRANSLATION)
        ROS_WARN_STREAM(motion_mag);

    if(!fuse_point_cloud)
    {
      goto publishing;
    }
    if(!continuous_mode)
    {
      fuse_point_cloud = false;
    }
    if (motion_mag < DEFAULT_MINIMUM_TRANSLATION)
    {
        ROS_DEBUG_STREAM("Camera motion below threshold");
        goto publishing;
    }
    pcl::transformPointCloud (cloud, cloud_transformed, fusion_frame_T_camera);

    for(auto point:cloud_transformed)
    // if( point.z>bounding_box[4] && point.z<bounding_box[5] )
    if(point.x>bounding_box[0] && point.x<bounding_box[1] && point.y>bounding_box[2] && point.y<bounding_box[3] && point.z>bounding_box[4] && point.z<bounding_box[5] )
        temp.push_back(point);
    

    if(combined_pcl.points.size())
    {
      if(USE_ICP)
      {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new PointCloud<PointXYZRGB>);
          Eigen::Matrix4f final_transform = Eigen::Matrix4f::Identity();
          pairAlign (temp.makeShared(),combined_pcl.makeShared(),output,final_transform,downsample);
          std::cout<<"ICP Successfull.."<<std::endl;
          combined_pcl.clear();
          for(auto x:output->points)
              combined_pcl.points.push_back(x);      
      }
      else
      {
        combined_pcl=combined_pcl+temp;
        std::cout<<"Point Clouds Combined.."<<std::endl;
        if(downsample)
          combined_pcl=PCLUtilities::downsample(combined_pcl,leaf_size);
      }
    }

    if(combined_pcl.points.size()==0)
        combined_pcl=temp;
    fusion_frame_T_camera_prev_=fusion_frame_T_camera;
    
    publishing: ;

    temp.clear();
    for(auto point:combined_pcl)
        if(point.x>bounding_box[0] && point.x<bounding_box[1] && point.y>bounding_box[2] && point.y<bounding_box[3] && point.z>bounding_box[4] && point.z<bounding_box[5] )
            temp.push_back(point);

    combined_pcl=temp;

    pcl::PCLPointCloud2* cloud_output = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_output);
    pcl::toPCLPointCloud2(combined_pcl, *cloud_output);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_output, output);   
    output.is_bigendian = false;
    output.header.seq=1;
    output.header.stamp=ros::Time::now();
    output.header.frame_id=fusion_frame_;
    output.height = combined_pcl.height;
    output.width = combined_pcl.width; 
    publish_cloud.publish (output);
}

bool PclFusion::savePointCloud(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
	// ROS_WARN("I heard: [%s]", msg->data.c_str());
  res.success=true;
  PCLUtilities::pclToXYZ<PointXYZRGB>(combined_pcl,"/home/rex/Desktop/REX_WORK_SPACE/Test_WS/REX/CGAL/Data/test.xyz");
  PCLUtilities::PclToPcd<PointXYZRGB>("/home/rex/Desktop/REX_WORK_SPACE/Test_WS/REX/CGAL/Data/test.pcd",combined_pcl);//Hard coded for debugging purposes...
  std::cout<<"Fusion Done..."<<std::endl;
  return true;
}

bool PclFusion::resetFusion(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  res.success=true;
  combined_pcl.clear();
  //Might need to make the current transformation matrix as identity....TODO
  return true;
}

bool PclFusion::capturePointCloud(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  res.success = true;
  fuse_point_cloud = true;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void PclFusion::pairAlign (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{

  PointCloud<PointXYZRGB>::Ptr src (new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr tgt (new PointCloud<PointXYZRGB>);
  pcl::VoxelGrid<PointXYZRGB> grid;
  std::cout<<"Cleaning Point Clouds..."<<std::endl;
  if (downsample)
  {
    grid.setLeafSize (leaf_size,leaf_size,leaf_size);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);
    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);

    // cleanPointCloud(cloud_src,src);
    // cleanPointCloud(cloud_tgt,tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }
  std::cout<<"Computing Normals..."<<std::endl;

  // Compute surface normals and curvature
  PointCloud<PointNormal>::Ptr points_with_normals_src (new PointCloud<PointNormal>);
  PointCloud<PointNormal>::Ptr points_with_normals_tgt (new PointCloud<PointNormal>);

  pcl::NormalEstimation<PointXYZRGB, PointNormal> norm_est;
  pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (neighbors);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //The following commented codes not linking, TODO.

  // std::vector<int> aux_indices;
  // removeNaNFromPointCloud (*src, *src, aux_indices);
  // removeNaNNormalsFromPointCloud (*src, *src, aux_indices);

  // std::vector<int> aux_indices2;
  // removeNaNFromPointCloud (*tgt, *tgt, aux_indices2);
  // removeNaNNormalsFromPointCloud (*tgt, *tgt, aux_indices2);


  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);
  std::cout<<"Aligning Point Clouds..."<<std::endl;
  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
  reg.setTransformationEpsilon (1e-9);
  // Set the maximum distance between two correspondences (src<->tgt)
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.005); //TODO: Need to update this later.... Might be too tight
  reg.setEuclideanFitnessEpsilon (1);
  reg.setRANSACOutlierRejectionThreshold (0.05);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);
// Run the same optimization in a loop
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  pcl::PointCloud<PointNormal>::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (max_icp_iterations);
  for (int i = 0; i < max_rounds; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);
        //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
    if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.0002);    
    prev = reg.getLastIncrementalTransformation ();
  }
  targetToSource = Ti.inverse();
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
  //add the source to the transformed target
   *output += *cloud_src;
   final_transform = targetToSource;
   std::cout<<"End of the align function..."<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_node");
    ros::NodeHandle pnh("~");
    string fusion_frame="";
    pnh.param<std::string>("fusion_frame", fusion_frame, "fusion_frame");
    std::vector<double> bounding_box;
    pnh.param("bounding_box", bounding_box, std::vector<double>());
    PclFusion pf(pnh,fusion_frame,bounding_box);    
    ros::spin();
    return 0;
}
