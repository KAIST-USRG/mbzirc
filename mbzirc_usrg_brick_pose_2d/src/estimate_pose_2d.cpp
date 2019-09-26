#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
// #include <pcl/features/organized_edge_detection.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>



//ROS include
// image
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <vector>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

//#include "estimate_pose_2d/block_pose.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"

typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointXYZRGBA RGBAPoint;
typedef pcl::PointCloud<RGBAPoint> RGBAPointCloud;

static int _h_min1;
static int _h_max1;
static int _h_min2;
static int _h_max2;

static int _s_min;
static int _s_max;
static int _v_min;
static int _v_max;

static int _edge_low_thres;
static int _edge_high_thres;

static double _error_thresh;
static double _short_real_length;
static double _long_real_length;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace cv;
using namespace std;

int thresh = 200;
int max_thresh = 255;
const char* source_window = "Source image";
const char* corners_window = "Corners detected";

class EstimatePose
{
private:
  bool image_updated = false;
  bool points_updated = false;
  int flag = 1;

  cv::Mat image;
  VPointCloud depth_points;
  VPointCloud clustered_points;
  ros::Subscriber depthpoints_sub;
  ros::Subscriber flag_sub;
  image_transport::Subscriber image_sub;
  ros::Publisher filtered_points_pub;
  ros::Publisher vertex_points_pub;
  // ros::Publisher vertex_rgb_pub;
  ros::Publisher lines_points_pub;

  ros::Publisher edge_points_pub;
  ros::Subscriber rgbd_points_sub;

  ros::Publisher cluster_points_pub;
  ros::Publisher center_point_pub;

  ros::Publisher block_pose_pub;

  float P_mat[12] = {615.4674072265625, 0.0, 319.95697021484375, 0.0, 0.0, 615.7725219726562, 245.20480346679688, 0.0, 0.0, 0.0, 1.0, 0.0};


public:
  EstimatePose(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  {
    priv_nh.param("h_min1", _h_min1, 0);
    ROS_INFO("h_min1: %d", _h_min1);
    priv_nh.param("h_max1", _h_max1, 15);
    ROS_INFO("h_max1: %d", _h_max1);
    priv_nh.param("h_min2", _h_min2, 110);
    ROS_INFO("h_min2: %d", _h_min2);
    priv_nh.param("h_max2", _h_max2, 180);
    ROS_INFO("h_max2: %d", _h_max2);

    priv_nh.param("s_min", _s_min, 10);
    ROS_INFO("s_min: %d", _s_min);
    priv_nh.param("s_max", _s_max, 255);
    ROS_INFO("s_max: %d", _s_max);
    priv_nh.param("v_min", _v_min, 30);
    ROS_INFO("v_min: %d", _v_min);
    priv_nh.param("v_max", _v_max, 255);
    ROS_INFO("v_max: %d", _v_max);

    priv_nh.param("edge_low_thres", _edge_low_thres, 50);
    ROS_INFO("edge_low_thres: %d", _edge_low_thres);
    priv_nh.param("edge_high_thres", _edge_high_thres, 100);
    ROS_INFO("edge_high_thres: %d", _edge_high_thres);

    priv_nh.param("error_thresh", _error_thresh, 0.15);
    ROS_INFO("error_thresh: %f", _error_thresh);

    priv_nh.param("short_real_length", _short_real_length, 0.1);
    ROS_INFO("short_real_length: %f", _short_real_length);
    priv_nh.param("long_real_length", _long_real_length, 0.32);
    ROS_INFO("long_real_length: %f", _long_real_length);


    // flag_sub = nh.subscribe("real_sense/flag", 10, 
    //                         &EstimatePose::flagCallback, this,
    //                         ros::TransportHints().tcpNoDelay(true));

    depthpoints_sub = nh.subscribe("/camera/depth_registered/points", 10,
                                  &EstimatePose::pointsCallback, this,
                                  ros::TransportHints().tcpNoDelay(true));

    image_transport::ImageTransport it(nh);
    image_sub = it.subscribe("/camera/color/image_rect_color", 1, 
                              &EstimatePose::imageCallback, this);
    
    filtered_points_pub = nh.advertise<VPointCloud>("/filtered_points", 10);

    vertex_points_pub = nh.advertise<VPointCloud>("/vertex_points", 10);
    // vertex_rgb_pub = nh.advertise<RGBPointCloud>("/vertex_rgb", 10);
    lines_points_pub = nh.advertise<VPointCloud>("/lines_points", 10);


    // edge_points_pub = nh.advertise<RGBAPointCloud>("/edge_points",10);
    // rgbd_points_sub = nh.subscribe("/camera/depth_registered/points", 10,
    //                               &EstimatePose::edgeDetectionPCL, this,
    //                               ros::TransportHints().tcpNoDelay(true));

    cluster_points_pub = nh.advertise<VPointCloud>("/cluster_points",10);
    center_point_pub = nh.advertise<VPointCloud>("/center_point",10);

    block_pose_pub = nh.advertise<geometry_msgs::Pose>("real_sense/block_pose", 10);


  }
  ~EstimatePose(){}
  
  pcl::visualization::PCLVisualizer::Ptr normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
  {
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
  }

  void clusteringPCL(cv::Mat& HSVmask)
  {
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (depth_points.makeShared());
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      // std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int max_num_points_on_hsvmask = -1;
    int max_num_points_idx = -1;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      int num_points_on_hsvmask = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        
        // find point cloud (HSV)
        double X = cloud_filtered->points[*pit].x;
        double Y = cloud_filtered->points[*pit].y;
        double Z = cloud_filtered->points[*pit].z;

        int x = ((P_mat[0]*X + P_mat[1]*Y + P_mat[2]*Z + P_mat[3])/(P_mat[8]*X + P_mat[9]*Y + P_mat[10]*Z + P_mat[11]));
        int y = ((P_mat[4]*X + P_mat[5]*Y + P_mat[6]*Z + P_mat[7])/(P_mat[8]*X + P_mat[9]*Y + P_mat[10]*Z + P_mat[11]));
    
        int img_width = HSVmask.cols;
        int img_height = HSVmask.rows;
        
        if (y>=0 && y<img_height && x >=0 && x<img_width)
        {
          if (HSVmask.at<uchar>(y,x)==255)
            num_points_on_hsvmask++;
        }

      }
      if (num_points_on_hsvmask > max_num_points_on_hsvmask)
      {
        max_num_points_on_hsvmask = num_points_on_hsvmask;
        max_num_points_idx = j;

      }
        
      // cloud_cluster->width = cloud_cluster->points.size ();
      // cloud_cluster->height = 1;
      // cloud_cluster->is_dense = true;

      // // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

      
      cloud_cluster->header = depth_points.header;
      
      j++;
    }

    if (max_num_points_idx != -1)
    {
      std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin () + max_num_points_idx;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        clustered_points.points.push_back(cloud_filtered->points[*pit]);

    }
    else
    {
      ROS_INFO("No cluster");
    }
    
    // ROS_INFO("%d", j);
  }

  void edgeDetectionPCL(const RGBAPointCloud::ConstPtr &scan)
  {
    VPointCloud cloud;
    cloud.header = scan->header;
    cloud.points.resize(scan->points.size());
    for (int i = 0;i<scan->points.size();i++)
    {
      cloud.points[i].x = scan->points[i].x;
      cloud.points[i].y = scan->points[i].y;
      cloud.points[i].z = scan->points[i].z;
    }
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud.makeShared());

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);

    // pcl::visualization::PCLVisualizer::Ptr viewer;
    // viewer = normalsVis(scan, cloud_normals);
    // ros::Rate loop_rate(10);
    // while (!viewer->wasStopped ())
    // {
    //   viewer->spinOnce (100);
    //   loop_rate.sleep();
    // }


    // pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> oed;
    // oed.setInputNormals (cloud_normals);
    // oed.setInputCloud (scan);
    // oed.setDepthDisconThreshold (0.02); // 2cm
    // oed.setMaxSearchNeighbors (50);
    // pcl::PointCloud<pcl::Label> labels;
    // std::vector<pcl::PointIndices> label_indices;
    // oed.compute (labels, label_indices);

    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
    //         occluded_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
    //         boundary_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
    //         high_curvature_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
    //         rgb_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);

    // pcl::copyPointCloud (*scan, label_indices[0].indices, *boundary_edges);
    // pcl::copyPointCloud (*scan, label_indices[1].indices, *occluding_edges);
    // pcl::copyPointCloud (*scan, label_indices[2].indices, *occluded_edges);
    // pcl::copyPointCloud (*scan, label_indices[3].indices, *high_curvature_edges);
    // pcl::copyPointCloud (*scan, label_indices[4].indices, *rgb_edges);

    // edge_points_pub.publish(*boundary_edges);
  }


  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      image = cv_bridge::toCvShare(msg, "bgr8")->image;
      image_updated = true;

      if (image_updated && points_updated)
      {
        processData();
        image_updated = false;
        points_updated = false;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
  void pointsCallback(const VPointCloud::ConstPtr &scan)
  {
    depth_points = *scan;

    // std::cout << scan->header.frame_id << std::endl;
    points_updated = true;
    if (image_updated && points_updated)
    {
      processData();
      image_updated = false;
      points_updated = false;
    }
  }

  // void flagCallback(const estimate_pose_2d::flag::ConstPtr& msg)
  // {
  //   flag = msg->flag;

  //   if (image_updated && points_updated)
  //   {
  //     if (flag == 1) // estimate pose 2d
  //       processData();
  //     else if (flag == 2)
  //       findMinDepth();
  //     image_updated = false;
  //     points_updated = false;
  //   }
  // }

  void detectHScolor(double minH, double maxH, double minS, double maxS, cv::Mat& mask)
  {
    cv::Mat hsv;
    cv::cvtColor(image,hsv,CV_BGR2HSV);
    // printf("(%d,%d,%d)\n", (int)hsv.at<cv::Vec3b>(300, 256)[0], (int)hsv.at<cv::Vec3b>(300, 256)[1], (int)hsv.at<cv::Vec3b>(300, 256)[2]);
    std::vector<cv::Mat> channels;
    cv::split(hsv,channels);
    cv::Mat mask1;
    cv::threshold(channels[0], mask1, maxH, 255, cv::THRESH_BINARY_INV);
    cv::Mat mask2;
    cv::threshold(channels[0], mask2, minH, 255, cv::THRESH_BINARY);
    cv::Mat hueMask;
    if (minH < maxH) hueMask = mask1 & mask2;
    else hueMask = mask1 | mask2;

    cv::threshold(channels[1], mask1, maxS, 255, cv::THRESH_BINARY_INV);
    cv::threshold(channels[1], mask2, minS, 255, cv::THRESH_BINARY);
    cv::Mat satMask;
    satMask = mask1 & mask2;

    mask = hueMask & satMask;
    
  }
  bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
  {
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/ 1e-8)
      return false;
    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true ; 
    
  }
  void setLabel(Mat& image, string str, vector<Point> contour)
  {
    int fontface = FONT_HERSHEY_SIMPLEX;
    double scale = 0.5;
    int thickness = 1;
    int baseline = 0;

    Size text = getTextSize(str, fontface, scale, thickness, &baseline);
    Rect r = boundingRect(contour);

    Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
    rectangle(image, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(200, 200, 200), FILLED);
    putText(image, str, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
  }
  
  double angle( Point pt1, Point pt2, Point pt0 )
  {
      double dx1 = pt1.x - pt0.x;
      double dy1 = pt1.y - pt0.y;
      double dx2 = pt2.x - pt0.x;
      double dy2 = pt2.y - pt0.y;
      return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
  }

  void detectEdge(int lowThreshold, int highThreshold, cv::Mat& detected_edges)
  {
    cv::Mat src_gray;
    cvtColor(image, src_gray, COLOR_BGR2GRAY);
    GaussianBlur( src_gray, detected_edges, Size( 3, 3 ), 0, 0 );
    Canny(detected_edges, detected_edges, lowThreshold, highThreshold, 3);


    // Hough Transform
    // Standard Hough Line Transform
    // vector<Vec2f> lines; // will hold the results of the detection
    // cv::Mat element5(7, 7, CV_8U, cv::Scalar(1));
    // morphologyEx( detected_edges, detected_edges, MORPH_CLOSE, element5 );
    // HoughLines(detected_edges, lines, 1, CV_PI/180, 100, 0, 0 ); // runs the actual detection
    // vector<Point> start_pt;
    // vector<Point> end_pt;
    // // Draw the lines
    // for( size_t i = 0; i < lines.size(); i++ )
    // {
    //     float rho = lines[i][0], theta = lines[i][1];
    //     Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a*rho, y0 = b*rho;
    //     pt1.x = cvRound(x0 + 1000*(-b));
    //     pt1.y = cvRound(y0 + 1000*(a));
    //     pt2.x = cvRound(x0 - 1000*(-b));
    //     pt2.y = cvRound(y0 - 1000*(a));
    //     line( image, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    //     start_pt.push_back(pt1);
    //     end_pt.push_back(pt2);
    // }
    // int img_width = image.cols;
    // int img_height = image.rows;
    // if (start_pt.size() < 1)
    //   return;
    // for (int i = 0;i<start_pt.size()-1;i++)
    // {
    //   Point2f intersec;
    //   bool a = intersection(start_pt[i], end_pt[i], start_pt[i+1], end_pt[i+1], intersec);
    //   if (a)
    //   {
    //     int y = (int)intersec.y;
    //     int x = (int)intersec.x;

    //     if (x >= 0 && x < img_width && y >=0 && y < img_height)
    //     {
    //       circle(image, Point(x, y), 5, Scalar(255, 0, 0), 5, 8);
    //       // image.at<cv::Vec3b>(x, y)[0] = 0;
    //       // image.at<cv::Vec3b>(x, y)[0] = 0;
    //       // image.at<cv::Vec3b>(x, y)[0] = 255;  
    //     }
    //   }
    // }
    // ROS_INFO("line size = %d", lines.size());
    // ROS_INFO("\n");
  
  
    // cvNamedWindow("result", WINDOW_AUTOSIZE );  
    // imshow( "result", image );  
    // waitKey(1);
  
  }

  bool detectRectVertex(cv::Mat& detected_edges, cv::Mat& rectangle_img, vector<Point>& poly)
  {
    cv::Mat morph;
    dilate(detected_edges, morph, Mat(), Point(-1,-1));

    size_t idx, i;
    vector<vector<Point>> contours;
    findContours(morph, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    
    Mat contours_img;
    cvtColor(detected_edges, contours_img, CV_GRAY2BGR);
    
    for (idx = 0;idx<contours.size();idx++)
    {
      bool draw = false;
      approxPolyDP(contours[idx], poly, arcLength(Mat(contours[idx]), true)*0.02, true);
      // convexHull(contours[idx], hull, false, true);
      for (i=0;i<poly.size();i++)
      {
        if( poly.size() == 4 &&
                    fabs(contourArea(Mat(poly))) > 1000 &&
                    isContourConvex(Mat(poly)) )
        {
            double maxCosine = 0;

            for( int j = 2; j < 5; j++ )
            {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(poly[j%4], poly[j-2], poly[j-1]));
                maxCosine = MAX(maxCosine, cosine);
            }

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if( maxCosine < 0.3 )
            {
              draw = true;
              // line(rectangle_img, poly[i], poly[(i+1)%poly.size()], Scalar(255,0,0) ,2);
            }
                
        }
      }
      if (draw)
      {
        // line(rectangle_img, poly[0], poly[(0+1)%poly.size()], Scalar(0,0,255) ,2);
        // line(rectangle_img, poly[1], poly[(1+1)%poly.size()], Scalar(0,255,0) ,2);
        // line(rectangle_img, poly[2], poly[(2+1)%poly.size()], Scalar(255,0,0) ,2);
        // line(rectangle_img, poly[3], poly[(3+1)%poly.size()], Scalar(0,255,255) ,2);
        circle(rectangle_img, poly[0], 5, Scalar(255,0,0), -1); //B
        circle(rectangle_img, poly[1], 5, Scalar(0,255,0), -1); //G
        circle(rectangle_img, poly[2], 5, Scalar(0,0,255), -1); //R 
        circle(rectangle_img, poly[3], 5, Scalar(0,255,255), -1); //Y
        return true;
      }

    }
    return false;
  }

  void lineFitting(const VPointCloud::Ptr &line_cloud, Eigen::VectorXf &coefficients_vec)
  {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<VPoint> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.001);

    seg.setInputCloud (line_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      ROS_WARN("[Laser] Could not estimate a planar model for the given dataset.");
      return;
    }

    // Copy coefficients to proper object for further filtering
    coefficients_vec(0) = coefficients->values[0];
    coefficients_vec(1) = coefficients->values[1];
    coefficients_vec(2) = coefficients->values[2];
    coefficients_vec(3) = coefficients->values[3];
    coefficients_vec(4) = coefficients->values[4];
    coefficients_vec(5) = coefficients->values[5];
  }

  void findMinDepth()
  {
    if (flag != 2)
      return;
    // find min depth
    double min_z = 10000.;
    for (int i = 0;i<depth_points.points.size();i++)
    {
      double z = depth_points.points[i].z;
      if (z > 0 && z < min_z)
        min_z = z;
    }

    // estimate_pose_2d::block_pose block_pose;
    // block_pose.x = 0, block_pose.y = 0, block_pose.z = 0;
    // block_pose.yaw = 0, block_pose.depth = min_z;
    // block_pose_pub.publish(block_pose);
    geometry_msgs::Pose block_pose;
    block_pose.position.x = 0, block_pose.position.y = 0, block_pose.position.z = min_z;
    double yaw = 0;
    tf::Matrix3x3 obs_mat;
    tf::Quaternion q_tf;

    obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
    obs_mat.getRotation(q_tf);
    block_pose.orientation.x = q_tf.getX();
    block_pose.orientation.y = q_tf.getY();
    block_pose.orientation.z = q_tf.getZ();
    block_pose.orientation.w = q_tf.getW();
    block_pose_pub.publish(block_pose);

  }

  void processData()
  {
    if (flag != 1)
      return;
    // object 2d detection in rgb image using color
    // convert RGB -> HSV
    
    cv::Mat detected_HS, detected_edges, mask;

    // HSV
    cv::Mat mask1, mask2;
    cv::Mat hsv;
    cv::cvtColor(image,hsv,CV_BGR2HSV);
    GaussianBlur( hsv, hsv, Size( 3, 3 ), 0, 0 );
    //detectHScolor(110, 180, 50, 255, detected_HS);
    cv::inRange(hsv, cv::Scalar(_h_min1, _s_min, _v_min), cv::Scalar(_h_max1, _s_max, _v_max), mask1);
    cv::inRange(hsv, cv::Scalar(_h_min2, _s_min, _v_min), cv::Scalar(_h_max2, _s_max, _v_max), mask2);

    detected_HS = mask1 | mask2;
    
    // Edge detection
    detectEdge(_edge_low_thres,_edge_high_thres,detected_edges);
    
  
    // bitwise_and(detected_HS,detected_edges, mask);
    bitwise_or(detected_HS,detected_edges, mask);
    imshow("HS", detected_HS);
    imshow("Canny Edge", detected_edges);
    imshow("mask", mask);
    waitKey(1);
    

    
    int img_width = image.cols;
    int img_height = image.rows;

    // 2d bbox -> point cloud plane RANSAC -> projection to 2d image -> contour detection -> vertex -> center, yaw

    // filter HSV
    int num_points = depth_points.size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->header = depth_points.header;
    for (int i = 0;i<num_points;i++)
    {
      double X = depth_points.points[i].x;
      double Y = depth_points.points[i].y;
      double Z = depth_points.points[i].z;
      int x = ((P_mat[0]*X + P_mat[1]*Y + P_mat[2]*Z + P_mat[3])/(P_mat[8]*X + P_mat[9]*Y + P_mat[10]*Z + P_mat[11]));
      int y = ((P_mat[4]*X + P_mat[5]*Y + P_mat[6]*Z + P_mat[7])/(P_mat[8]*X + P_mat[9]*Y + P_mat[10]*Z + P_mat[11]));
  
      // if (x>=x_LT && x<=x_LT+width && y>=y_LT && y<=y_LT+height)
      if (y>=0 && y<img_height && x >=0 && x<img_width)
      {
        if (mask.at<uchar>(y,x)==255)
          filtered_cloud->points.push_back(depth_points.points[i]);
      }
      
    }
    
    // filter HSV using clustering
    // clustered_points.points.clear();
    // clustered_points.header = depth_points.header;
    // clusteringPCL(detected_HS);
    
    // RANSAC for fitting plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    // seg.setInputCloud (clustered_points.makeShared());
    seg.setInputCloud (filtered_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return;
    }

    // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    //                                     << coefficients->values[1] << " "
    //                                     << coefficients->values[2] << " " 
    //                                     << coefficients->values[3] << std::endl;

    // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    // inlier
    // VPointCloud plane_points;
    // plane_points.header = clustered_points.header;
    // for (size_t i = 0; i < inliers->indices.size (); ++i)
    // {
    //   plane_points.points.push_back(clustered_points.points[inliers->indices[i]]);
    // }
    VPointCloud plane_points;
    plane_points.header = filtered_cloud->header;
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
      plane_points.points.push_back(filtered_cloud->points[inliers->indices[i]]);
    }
    

    // project points to plane
    VPointCloud projected_plane_points;
    projected_plane_points.header = plane_points.header;
    float A = coefficients->values[0]; float B = coefficients->values[1];
    float C = coefficients->values[2]; float D = coefficients->values[3];
    for (int i = 0;i<plane_points.points.size();i++)
    {
      float x = plane_points.points[i].x;
      float y = plane_points.points[i].y;
      float z = plane_points.points[i].z;
      float t = -(A*x + B*y + C*z + D)/(A*A + B*B + C*C);

      VPoint tmp;
      tmp.x = A*t + x, tmp.y = B*t + y, tmp.z = C*t + z;
      projected_plane_points.points.push_back(tmp);
    }
    filtered_points_pub.publish(projected_plane_points);

    ////////////////////// 1 ///////////////////////////
    // 1. find xmin xmax ymin ymax
    VPointCloud vertex_points;
    vertex_points.header = projected_plane_points.header;

     // estimate vertex
    // find min y, max y, min z, max z in cluster
    double min_y = 100000.;
    double max_y = -100000.;
    double min_x = 100000.;
    double max_x = -100000.;
    int min_y_idx = -1;
    int max_y_idx = -1;
    int min_x_idx = -1;
    int max_x_idx = -1;

    for (int i = 0;i<projected_plane_points.points.size();i++)
    {
      double X = projected_plane_points.points[i].x;
      double Y = projected_plane_points.points[i].y;
      double Z = projected_plane_points.points[i].z;

      if (Y < min_y)
      {
        min_y = Y;
        min_y_idx = i;
      }
      if (Y > max_y)
      {
        max_y = Y;
        max_y_idx = i;
      }
      if (X < min_x)
      {
        min_x = X;
        min_x_idx = i;
      }
      if (X > max_x)
      {
        max_x = X;
        max_x_idx = i;
      }
    }

    

    vertex_points.points.push_back(projected_plane_points.points[min_y_idx]); // top
    vertex_points.points.push_back(projected_plane_points.points[min_x_idx]); // left
    vertex_points.points.push_back(projected_plane_points.points[max_y_idx]); // bottomm
    vertex_points.points.push_back(projected_plane_points.points[max_x_idx]); // right

    // calculate error
    float error[4];
    VPoint vec;
    // ROS_INFO("%f %f", pcl::euclideanDistance(vertex_points.points[0], vertex_points.points[1]), pcl::euclideanDistance(vertex_points.points[1], vertex_points.points[2]));
    if (pcl::euclideanDistance(vertex_points.points[0], vertex_points.points[1]) > pcl::euclideanDistance(vertex_points.points[1], vertex_points.points[2]))
    {
      error[0] = fabs(_long_real_length - pcl::euclideanDistance(vertex_points.points[0], vertex_points.points[1]))/_long_real_length;
      error[1] = fabs(_short_real_length - pcl::euclideanDistance(vertex_points.points[1], vertex_points.points[2]))/_short_real_length;
      error[2] = fabs(_long_real_length - pcl::euclideanDistance(vertex_points.points[2], vertex_points.points[3]))/_long_real_length;
      error[3] = fabs(_short_real_length - pcl::euclideanDistance(vertex_points.points[3], vertex_points.points[0]))/_short_real_length;

      vec.x = vertex_points.points[0].x - vertex_points.points[1].x;
      vec.y = vertex_points.points[0].y - vertex_points.points[1].y;
      vec.z = vertex_points.points[0].z - vertex_points.points[1].z;
    }
    else
    {
      error[0] = fabs(_short_real_length - pcl::euclideanDistance(vertex_points.points[0], vertex_points.points[1]))/_short_real_length;
      error[1] = fabs(_long_real_length - pcl::euclideanDistance(vertex_points.points[1], vertex_points.points[2]))/_long_real_length;
      error[2] = fabs(_short_real_length - pcl::euclideanDistance(vertex_points.points[2], vertex_points.points[3]))/_short_real_length;
      error[3] = fabs(_long_real_length - pcl::euclideanDistance(vertex_points.points[3], vertex_points.points[0]))/_long_real_length;

      vec.x = vertex_points.points[1].x - vertex_points.points[2].x;
      vec.y = vertex_points.points[1].y - vertex_points.points[2].y;
      vec.z = vertex_points.points[1].z - vertex_points.points[2].z;
    }
    
    // ROS_INFO("error = (%f, %f, %f, %f)", error[0], error[1], error[2], error[3]);
    
    // 1 minmax
    // if (error[0] <= _error_thresh && error[1] <= _error_thresh && error[2] <= _error_thresh && error[3] <= _error_thresh)
    if (error[0] <= _error_thresh && error[1] <= _error_thresh && error[2] <= _error_thresh && error[3] <= _error_thresh)
    {
      // estimate center
      VPoint center;
      center.x = (projected_plane_points.points[min_y_idx].x + projected_plane_points.points[max_y_idx].x + projected_plane_points.points[min_x_idx].x + projected_plane_points.points[max_x_idx].x)/4.;
      center.y = (projected_plane_points.points[min_y_idx].y + projected_plane_points.points[max_y_idx].y + projected_plane_points.points[min_x_idx].y + projected_plane_points.points[max_x_idx].y)/4.;
      center.z = (projected_plane_points.points[min_y_idx].z + projected_plane_points.points[max_y_idx].z + projected_plane_points.points[min_x_idx].z + projected_plane_points.points[max_x_idx].z)/4.;
      vertex_points.points.push_back(center);

      // estimate yaw
      VPoint axis;
      axis.x = 1, axis.y = 0, axis.z = 0;
      double yaw = acos((vec.x*axis.x)/(sqrtf(vec.x*vec.x+vec.y*vec.y+vec.z*vec.z)*1));

      if (yaw > M_PI/2.)
        yaw = M_PI - yaw;
      else
        yaw = -yaw;
      // yaw = MIN(yaw, M_PI-yaw);
      ROS_INFO("1 (x,y,z)=(%f,%f,%f),yaw = %f deg", center.x, center.y, center.z, yaw*180/M_PI);
      
      // publish
      // estimate_pose_2d::block_pose block_pose;
      // block_pose.x = center.x;
      // block_pose.y = center.y;
      // block_pose.z = center.z;
      // block_pose.yaw = yaw;
      // block_pose_pub.publish(block_pose);

      geometry_msgs::Pose block_pose;
      block_pose.position.x = center.x, block_pose.position.y = center.y, block_pose.position.z = center.z;
      tf::Matrix3x3 obs_mat;
      tf::Quaternion q_tf;

      obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
      obs_mat.getRotation(q_tf);
      block_pose.orientation.x = q_tf.getX();
      block_pose.orientation.y = q_tf.getY();
      block_pose.orientation.z = q_tf.getZ();
      block_pose.orientation.w = q_tf.getW();
      block_pose_pub.publish(block_pose);

      vertex_points_pub.publish(vertex_points);
      return;
    }


    ////////////////////// 2 ///////////////////////////
    else
    {
      // use image rectangle detection point
      // Rectangle detection
      cv::Mat rectangle_img = Mat::zeros(image.size(), image.type());
      vector<Point> poly;

      bool detectVertex = detectRectVertex(detected_edges, rectangle_img, poly);
      imshow("rect_img", rectangle_img);
      waitKey(1);
      if (!detectVertex)
        return; 
      
      // avg vertex
      Point3d RV(0.,0.,0.), GV(0.,0.,0.), BV(0.,0.,0.), YV(0.,0.,0.);
      int R_size = 0, G_size = 0, B_size = 0, Y_size = 0;
      for (int i = 0;i<projected_plane_points.points.size();i++)
      {
        VPoint pt = projected_plane_points.points[i];
        double X = projected_plane_points.points[i].x;
        double Y = projected_plane_points.points[i].y;
        double Z = projected_plane_points.points[i].z;
        int x = ((P_mat[0]*X + P_mat[1]*Y + P_mat[2]*Z + P_mat[3])/(P_mat[8]*X + P_mat[9]*Y + P_mat[10]*Z + P_mat[11]));
        int y = ((P_mat[4]*X + P_mat[5]*Y + P_mat[6]*Z + P_mat[7])/(P_mat[8]*X + P_mat[9]*Y + P_mat[10]*Z + P_mat[11]));
    
        if (y>=0 && y<img_height && x >=0 && x<img_width)
        {
          if (rectangle_img.at<cv::Vec3b>(y,x)[0]==255 && rectangle_img.at<cv::Vec3b>(y,x)[1]==0 && rectangle_img.at<cv::Vec3b>(y,x)[2]==0)
            {BV.x += X, BV.y += Y, BV.z += Z; B_size++;}
          else if (rectangle_img.at<cv::Vec3b>(y,x)[0]==0 && rectangle_img.at<cv::Vec3b>(y,x)[1]==255 && rectangle_img.at<cv::Vec3b>(y,x)[2]==0)
            {GV.x += X, GV.y += Y, GV.z += Z; G_size++;}
          else if (rectangle_img.at<cv::Vec3b>(y,x)[0]==0 && rectangle_img.at<cv::Vec3b>(y,x)[1]==0 && rectangle_img.at<cv::Vec3b>(y,x)[2]==255)
            {RV.x += X, RV.y += Y, RV.z += Z; R_size++;}
          else if (rectangle_img.at<cv::Vec3b>(y,x)[0]==0 && rectangle_img.at<cv::Vec3b>(y,x)[1]==255 && rectangle_img.at<cv::Vec3b>(y,x)[2]==255)
            {YV.x += X, YV.y += Y, YV.z += Z; Y_size++;}
        }
      }

      Eigen::Vector3f V[4];
      if (B_size > 0)
      {
        V[0](0) = BV.x/(double)B_size;
        V[0](1) = BV.y/(double)B_size;
        V[0](2) = BV.z/(double)B_size;
      }    
      if (G_size > 0)
      {
        V[1](0) = GV.x/(double)G_size;
        V[1](1) = GV.y/(double)G_size;
        V[1](2) = GV.z/(double)G_size;
      }
      
      if (R_size > 0)
      {
        V[2](0) = RV.x/(double)R_size;
        V[2](1) = RV.y/(double)R_size;
        V[2](2) = RV.z/(double)R_size;
      }
      
      if (Y_size > 0)
      {
        V[3](0) = YV.x/(double)Y_size;
        V[3](1) = YV.y/(double)Y_size;
        V[3](2) = YV.z/(double)Y_size;
      }
      

      // // line fitting
      // VPointCloud Rline, Gline, Bline, Yline;
      // Rline.header = projected_plane_points.header;
      // Gline.header = projected_plane_points.header;
      // Bline.header = projected_plane_points.header;
      // Yline.header = projected_plane_points.header;

      // for (int i = 0;i<projected_plane_points.points.size();i++)
      // {
      //   VPoint pt = projected_plane_points.points[i];
      //   double X = projected_plane_points.points[i].x;
      //   double Y = projected_plane_points.points[i].y;
      //   double Z = projected_plane_points.points[i].z;
      //   int x = ((P_mat[0]*X + P_mat[1]*Y + P_mat[2]*Z + P_mat[3])/(P_mat[8]*X + P_mat[9]*Y + P_mat[10]*Z + P_mat[11]));
      //   int y = ((P_mat[4]*X + P_mat[5]*Y + P_mat[6]*Z + P_mat[7])/(P_mat[8]*X + P_mat[9]*Y + P_mat[10]*Z + P_mat[11]));
    
      //   if (y>=0 && y<img_height && x >=0 && x<img_width)
      //   {
      //     if (rectangle_img.at<cv::Vec3b>(y,x)[0]==0 && rectangle_img.at<cv::Vec3b>(y,x)[1]==0 && rectangle_img.at<cv::Vec3b>(y,x)[2]==255)
      //       Rline.points.push_back(pt);
      //     else if (rectangle_img.at<cv::Vec3b>(y,x)[0]==0 && rectangle_img.at<cv::Vec3b>(y,x)[1]==255 && rectangle_img.at<cv::Vec3b>(y,x)[2]==0)
      //       Gline.points.push_back(pt);
      //     else if (rectangle_img.at<cv::Vec3b>(y,x)[0]==255 && rectangle_img.at<cv::Vec3b>(y,x)[1]==0 && rectangle_img.at<cv::Vec3b>(y,x)[2]==0)
      //       Bline.points.push_back(pt);
      //     else if (rectangle_img.at<cv::Vec3b>(y,x)[0]==0 && rectangle_img.at<cv::Vec3b>(y,x)[1]==255 && rectangle_img.at<cv::Vec3b>(y,x)[2]==255)
      //       Yline.points.push_back(pt);
      //   }
      // }

      // Eigen::VectorXf coefficients_R(6);
      // lineFitting(Rline.makeShared(), coefficients_R);

      // Eigen::VectorXf coefficients_G(6);
      // lineFitting(Gline.makeShared(), coefficients_G);

      // Eigen::VectorXf coefficients_B(6);
      // lineFitting(Bline.makeShared(), coefficients_B);

      // Eigen::VectorXf coefficients_Y(6);
      // lineFitting(Yline.makeShared(), coefficients_Y);
      // lines_points_pub.publish(Rline);

      // Eigen::Vector4f V[4]; // VU, VL, VD, VR 
      // pcl::lineWithLineIntersection(coefficients_R, coefficients_G, V[0]);
      // pcl::lineWithLineIntersection(coefficients_G, coefficients_B, V[1]);
      // pcl::lineWithLineIntersection(coefficients_B, coefficients_Y, V[2]);
      // pcl::lineWithLineIntersection(coefficients_Y, coefficients_R, V[3]);

      VPointCloud vertex_points;
      vertex_points.header = plane_points.header;

      // vertex cloud
      for (int i = 0;i<4;i++)
      {
        VPoint tmp;
        tmp.x = V[i](0), tmp.y = V[i](1), tmp.z = V[i](2);
        vertex_points.points.push_back(tmp);
      }
      
      // calculate error
      float error[4];
      VPoint vec; //ref
      // ROS_INFO("(%f,%f,%f) (%f,%f,%f) (%f,%f,%f) (%f,%f,%f)", V[0](0), V[0](1), V[0](2), V[1](0), V[1](1), V[1](2), V[2](0), V[2](1), V[2](2), V[3](0), V[3](1), V[3](2));
      // ROS_INFO("%f %f", pcl::euclideanDistance(vertex_points.points[0], vertex_points.points[1]), pcl::euclideanDistance(vertex_points.points[1], vertex_points.points[2]));
      if (pcl::euclideanDistance(vertex_points.points[0], vertex_points.points[1]) > pcl::euclideanDistance(vertex_points.points[1], vertex_points.points[2]))
      {
        error[0] = fabs(_long_real_length - pcl::euclideanDistance(vertex_points.points[0], vertex_points.points[1]))/_long_real_length;
        error[1] = fabs(_short_real_length - pcl::euclideanDistance(vertex_points.points[1], vertex_points.points[2]))/_short_real_length;
        error[2] = fabs(_long_real_length - pcl::euclideanDistance(vertex_points.points[2], vertex_points.points[3]))/_long_real_length;
        error[3] = fabs(_short_real_length - pcl::euclideanDistance(vertex_points.points[3], vertex_points.points[0]))/_short_real_length;

        vec.x = vertex_points.points[0].x - vertex_points.points[1].x;
        vec.y = vertex_points.points[0].y - vertex_points.points[1].y;
        vec.z = vertex_points.points[0].z - vertex_points.points[1].z;
        if (vec.y > 0)
          {vec.x = -vec.x, vec.y = -vec.y, vec.z = -vec.z;}
      }
      else
      {
        error[0] = fabs(_short_real_length - pcl::euclideanDistance(vertex_points.points[0], vertex_points.points[1]))/_short_real_length;
        error[1] = fabs(_long_real_length - pcl::euclideanDistance(vertex_points.points[1], vertex_points.points[2]))/_long_real_length;
        error[2] = fabs(_short_real_length - pcl::euclideanDistance(vertex_points.points[2], vertex_points.points[3]))/_short_real_length;
        error[3] = fabs(_long_real_length - pcl::euclideanDistance(vertex_points.points[3], vertex_points.points[0]))/_long_real_length;

        vec.x = vertex_points.points[1].x - vertex_points.points[2].x;
        vec.y = vertex_points.points[1].y - vertex_points.points[2].y;
        vec.z = vertex_points.points[1].z - vertex_points.points[2].z;
        if (vec.y > 0)
          {vec.x = -vec.x, vec.y = -vec.y, vec.z = -vec.z;}
      }
      
      // ROS_INFO("error = (%f, %f, %f, %f)", error[0], error[1], error[2], error[3]);
      
      // print
      if (error[0] <= _error_thresh && error[1] <= _error_thresh && error[2] <= _error_thresh && error[3] <= _error_thresh)
      {
        // estimate center
        Eigen::Vector3f center_vec(3,1);
        center_vec = (V[0] + V[1] + V[2] + V[3])/4.;

        // estimate yaw
        Eigen::Vector3f axis(3,1);
        axis << 1., 0., 0.;
        VPoint center;
        center.x = center_vec(0), center.y = center_vec(1), center.z = center_vec(2);
        vertex_points.points.push_back(center);
        

        // estimate center
        // Eigen::Vector4f center_4;
        // center_4 = (V[0] + V[1] + V[2] + V[3])/4.;
        // VPoint center;
        // center.x = center_4(0), center.y = center_4(1), center.z = center_4(2);
        // vertex_points.points.push_back(center);
        
        // // estimate yaw
        // Eigen::Vector3f axis(3,1);
        // axis << 1., 0., 0.;
        // Eigen::Vector3f center_vec(3,1);
        // center_vec << vec.x, vec.y, vec.z;

        // std::cout << axis*center_vec << std::endl;
        double yaw = acos((vec.x)/(1*sqrtf(vec.x*vec.x+vec.y*vec.y+vec.z*vec.z)));
        if (yaw > M_PI/2.)
          yaw = M_PI - yaw;
        else
          yaw = -yaw;
        // yaw = MIN(yaw, M_PI-yaw);
        
        ROS_INFO("2 (x,y,z)=(%f,%f,%f),yaw = %f deg", center.x, center.y, center.z, yaw*180/M_PI);

        // publish
        // estimate_pose_2d::block_pose block_pose;
        // block_pose.x = center.x;
        // block_pose.y = center.y;
        // block_pose.z = center.z;
        // block_pose.yaw = yaw;
        // block_pose_pub.publish(block_pose);

        geometry_msgs::Pose block_pose;
        block_pose.position.x = center.x, block_pose.position.y = center.y, block_pose.position.z = center.z;
        tf::Matrix3x3 obs_mat;
        tf::Quaternion q_tf;

        obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
        obs_mat.getRotation(q_tf);
        block_pose.orientation.x = q_tf.getX();
        block_pose.orientation.y = q_tf.getY();
        block_pose.orientation.z = q_tf.getZ();
        block_pose.orientation.w = q_tf.getW();
        block_pose_pub.publish(block_pose);
        
        vertex_points_pub.publish(vertex_points);
        return;
      }
          // find vertex using clusters that have diag len dist with centroid
      else
      {
        // calculate centroid
        Eigen::Vector4f centroid_vec;
        pcl::compute3DCentroid (projected_plane_points, centroid_vec);
        VPointCloud centroid_cloud;
        centroid_cloud.header = projected_plane_points.header;
        VPoint centroid;
        centroid.x = centroid_vec(0), centroid.y = centroid_vec(1), centroid.z = centroid_vec(2);
        centroid_cloud.points.push_back(centroid);
        
        VPointCloud vertex_cluster_cloud;
        vertex_cluster_cloud.header = projected_plane_points.header;

        // find clusters that have diag len dist with centroid
        double diag_length = sqrtf(_long_real_length*_long_real_length+_short_real_length*_short_real_length)/2.;
        for (int i = 0;i<projected_plane_points.points.size();i++)
        {
          double dist = pcl::euclideanDistance(projected_plane_points.points[i], centroid);
          double thres = diag_length/20.; 
          if (dist >= diag_length - thres)
            vertex_cluster_cloud.points.push_back(projected_plane_points.points[i]);
        }
        cluster_points_pub.publish(vertex_cluster_cloud);


        double centroid_x = centroid_vec(0);
        double centroid_y = centroid_vec(1);
        
        double top_right_max_dist = -1;
        double top_right_max_dist_idx = -1;

        double bottom_right_max_dist = -1;
        double bottom_right_max_dist_idx = -1;
        
        double top_left_max_dist = -1;
        double top_left_max_dist_idx = -1;

        double bottom_left_max_dist = -1;
        double bottom_left_max_dist_idx = -1;
        
        for (int i = 0;i<vertex_cluster_cloud.points.size();i++)
        {
          double x = vertex_cluster_cloud.points[i].x;
          double y = vertex_cluster_cloud.points[i].y;
          double z = vertex_cluster_cloud.points[i].z;

          double dist = pcl::euclideanDistance(vertex_cluster_cloud.points[i], centroid);

          if (x > centroid_x)
          {
            if (y < centroid_y) // top right
            {
              if (dist > top_right_max_dist)
              {
                top_right_max_dist = dist;
                top_right_max_dist_idx = i;
              }
            }
            else // bottom right
            {
              if (dist > bottom_right_max_dist)
              {
                bottom_right_max_dist = dist;
                bottom_right_max_dist_idx = i;
              }
            }
          }
          else
          {
            if (y < centroid_y) // top left
            {
              if (dist > top_left_max_dist)
              {
                top_left_max_dist = dist;
                top_left_max_dist_idx = i;
              }
            }
            else // bottom left
            {
              if (dist > bottom_left_max_dist)
              {
                bottom_left_max_dist = dist;
                bottom_left_max_dist_idx = i;
              }
            }
          }
        }

        vertex_points.points.clear();
        if (top_right_max_dist_idx != -1)
          vertex_points.points.push_back(vertex_cluster_cloud[top_right_max_dist_idx]);
        if (bottom_right_max_dist_idx != -1)
          vertex_points.points.push_back(vertex_cluster_cloud[bottom_right_max_dist_idx]);
        if (bottom_left_max_dist_idx != -1)
          vertex_points.points.push_back(vertex_cluster_cloud[bottom_left_max_dist_idx]);
        if (top_left_max_dist_idx != -1)
          vertex_points.points.push_back(vertex_cluster_cloud[top_left_max_dist_idx]);
        

        VPoint vec;
        int vertex_points_size = vertex_points.points.size();
        bool detect_realiable_line = false;
        int detect_line_type = 0; // 1: long line, 2: short line, 3: diag_line

        VPointCloud find_yaw_angle_points;
        find_yaw_angle_points.header = vertex_points.header;
        VPoint ref;

        if (vertex_points_size >= 2)
        {
          for (int i = 0;i<vertex_points_size;i++)
          {
            if (detect_line_type==1)
              continue;
            double dist = pcl::euclideanDistance(vertex_points.points[i], vertex_points.points[(i+1)%vertex_points_size]);
            double long_len_error = fabs(_long_real_length - dist)/_long_real_length;
            double short_len_error = fabs(_short_real_length - dist)/_short_real_length;
            double diag_len_error = fabs(diag_length*2.-dist)/(diag_length*2.);

            if (long_len_error < _error_thresh)
            {
              detect_realiable_line = true;
              detect_line_type = 1;
              vec.x = vertex_points.points[i].x - vertex_points.points[(i+1)%vertex_points_size].x;
              vec.y = vertex_points.points[i].y - vertex_points.points[(i+1)%vertex_points_size].y;
              vec.z = vertex_points.points[i].z - vertex_points.points[(i+1)%vertex_points_size].z;
              if (vec.y > 0)
                {vec.x = -vec.x, vec.y = -vec.y, vec.z = -vec.z;}
              find_yaw_angle_points.points.push_back(vertex_points.points[i]);
              find_yaw_angle_points.points.push_back(vertex_points.points[(i+1)%vertex_points_size]);
            }
            else if (short_len_error < _error_thresh)
            {
              detect_realiable_line = true;
              detect_line_type = 2;
              vec.x = vertex_points.points[i].x - vertex_points.points[(i+1)%vertex_points_size].x;
              vec.y = vertex_points.points[i].y - vertex_points.points[(i+1)%vertex_points_size].y;
              vec.z = vertex_points.points[i].z - vertex_points.points[(i+1)%vertex_points_size].z;
              if (vec.y > 0)
                {vec.x = -vec.x, vec.y = -vec.y, vec.z = -vec.z;}
              find_yaw_angle_points.points.push_back(vertex_points.points[i]);
              find_yaw_angle_points.points.push_back(vertex_points.points[(i+1)%vertex_points_size]);
            }
            else if (diag_len_error < _error_thresh)
            {
              detect_realiable_line = true;
              detect_line_type = 3;
              vec.x = vertex_points.points[i].x - vertex_points.points[(i+1)%vertex_points_size].x;
              vec.y = vertex_points.points[i].y - vertex_points.points[(i+1)%vertex_points_size].y;
              vec.z = vertex_points.points[i].z - vertex_points.points[(i+1)%vertex_points_size].z;

              find_yaw_angle_points.points.push_back(vertex_points.points[i]);
              find_yaw_angle_points.points.push_back(vertex_points.points[(i+1)%vertex_points_size]);

              ref = vertex_points.points[(i+1)%vertex_points_size];
            }
          }
        }
        
        if (detect_realiable_line)
        {
          find_yaw_angle_points.points.push_back(centroid);
          // estimate yaw
          VPoint axis;
          axis.x = 1, axis.y = 0, axis.z = 0;
          double yaw = acos((vec.x*axis.x)/(sqrtf(vec.x*vec.x+vec.y*vec.y+vec.z*vec.z)*1));
          
          if (detect_line_type == 2) // short line
          {
            if (yaw > M_PI/2.)
              yaw = M_PI - yaw;
            else
              yaw += M_PI/2.;  
          }

          else if (detect_line_type == 3) // diag line
          {
            double alpha = atan(_short_real_length/_long_real_length);
            
            // rotate +alpha
            VPoint rotated_vec1;
            rotated_vec1.x = vec.x*cos(alpha) + vec.y*sin(alpha) + ref.x;
            rotated_vec1.y = -vec.x*sin(alpha) + vec.y*cos(alpha) + ref.y;
            rotated_vec1.z = ref.z;

            // rotate -alpha
            VPoint rotated_vec2;
            rotated_vec2.x = vec.x*cos(alpha) - vec.y*sin(alpha) + ref.x;
            rotated_vec2.y = vec.x*sin(alpha) + vec.y*cos(alpha) + ref.y;
            rotated_vec2.z = ref.z;

            double min_dist_1 = 10000.;
            double min_dist_2 = 10000.;

            for (int i = 0;i<projected_plane_points.points.size();i++)
            {
              double dist1 = pcl::euclideanDistance(projected_plane_points.points[i], rotated_vec1);
              double dist2 = pcl::euclideanDistance(projected_plane_points.points[i], rotated_vec2);

              if (dist1 < min_dist_1)
                min_dist_1 = dist1;
              if (dist2 < min_dist_2)
                min_dist_2 = dist2;
            }

            if (min_dist_1 <= min_dist_2)
            {
              vec.x = rotated_vec1.x - ref.x;
              vec.y = rotated_vec1.y - ref.y;
              vec.z = rotated_vec1.z - ref.z;
              // find_yaw_angle_points.points.push_back(rotated_vec1);
            }
              
            else
            {
              vec.x = rotated_vec2.x - ref.x;
              vec.y = rotated_vec2.y - ref.y;
              vec.z = rotated_vec2.z - ref.z;
              // find_yaw_angle_points.points.push_back(rotated_vec2);
            }
            vertex_points_pub.publish(find_yaw_angle_points);
            if (vec.y > 0)
                {vec.x = -vec.x, vec.y = -vec.y, vec.z = -vec.z;}

            yaw = acos((vec.x*axis.x)/(sqrtf(vec.x*vec.x+vec.y*vec.y+vec.z*vec.z)*1));
          }
          if (yaw > M_PI/2.)
            yaw = M_PI - yaw;
          else
            yaw = -yaw;
          ROS_INFO("3 detect_line_type = %d, (x,y,z)=(%f,%f,%f),yaw = %f deg",detect_line_type, centroid.x, centroid.y, centroid.z, yaw*180/M_PI);

          // publish
          // estimate_pose_2d::block_pose block_pose;
          // block_pose.x = centroid.x;
          // block_pose.y = centroid.y;
          // block_pose.z = centroid.z;
          // block_pose.yaw = yaw;
          // block_pose_pub.publish(block_pose);

          geometry_msgs::Pose block_pose;
          block_pose.position.x = centroid.x, block_pose.position.y = centroid.y, block_pose.position.z = centroid.z;
          tf::Matrix3x3 obs_mat;
          tf::Quaternion q_tf;

          obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
          obs_mat.getRotation(q_tf);
          block_pose.orientation.x = q_tf.getX();
          block_pose.orientation.y = q_tf.getY();
          block_pose.orientation.z = q_tf.getZ();
          block_pose.orientation.w = q_tf.getW();
          block_pose_pub.publish(block_pose);

          return;  
        }
        else
        {
          ROS_INFO("not realiable");
        }
        
        
      }
      
    }




    

        
  }
  
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimate_pose_2d");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // cv::namedWindow("view");
  // cv::startWindowThread();
  EstimatePose estimatepose(nh, priv_nh);
  ros::spin();
  // cv::destroyWindow("view");

  return 0;
}
