/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <chrono>
#include <unordered_map> 
#include <queue>
#include <fstream>
#include <thread>
#include <ctime>
#include <bits/stdc++.h>

/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>
#include <omp.h>


#if defined(__SSE2__)
#include <xmmintrin.h>
#endif

#if defined(__AVX__)
#include <immintrin.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>


/** A helper struct to apply an SO3 or SE3 transform to a 3D point.
  * Supports single and double precision transform matrices. */
template<typename Scalar>
struct Transformer
{
  const Eigen::Matrix<Scalar, 4, 4>& tf;

  /** Construct a transformer object.
    * The transform matrix is captured by const reference. Make sure that it does not go out of scope before this
    * object does. */
  Transformer (const Eigen::Matrix<Scalar, 4, 4>& transform) : tf (transform) { };

  /** Apply SO3 transform (top-left corner of the transform matrix).
    * \param[in] src input 3D point (pointer to 3 floats)
    * \param[out] tgt output 3D point (pointer to 4 floats), can be the same as input. The fourth element is set to 0. */
  void so3 (const float* src, float* tgt) const
  {
    const Scalar p[3] = { src[0], src[1], src[2] };  // need this when src == tgt
    tgt[0] = static_cast<float> (tf (0, 0) * p[0] + tf (0, 1) * p[1] + tf (0, 2) * p[2]);
    tgt[1] = static_cast<float> (tf (1, 0) * p[0] + tf (1, 1) * p[1] + tf (1, 2) * p[2]);
    tgt[2] = static_cast<float> (tf (2, 0) * p[0] + tf (2, 1) * p[1] + tf (2, 2) * p[2]);
    tgt[3] = 0;
  }

  /** Apply SE3 transform.
    * \param[in] src input 3D point (pointer to 3 floats)
    * \param[out] tgt output 3D point (pointer to 4 floats), can be the same as input. The fourth element is set to 1. */
  void se3 (const float* src, float* tgt) const
  {
    const Scalar p[3] = { src[0], src[1], src[2] };  // need this when src == tgt
    tgt[0] = static_cast<float> (tf (0, 0) * p[0] + tf (0, 1) * p[1] + tf (0, 2) * p[2] + tf (0, 3));
    tgt[1] = static_cast<float> (tf (1, 0) * p[0] + tf (1, 1) * p[1] + tf (1, 2) * p[2] + tf (1, 3));
    tgt[2] = static_cast<float> (tf (2, 0) * p[0] + tf (2, 1) * p[1] + tf (2, 2) * p[2] + tf (2, 3));
    tgt[3] = 1;
  }
};

#if defined(__SSE2__)

/** Optimized version for single-precision transforms using SSE2 intrinsics. */
template<>
struct Transformer<float>
{
  /// Columns of the transform matrix stored in XMM registers.
  __m128 c[4];

  Transformer(const Eigen::Matrix4f& tf)
  {
    for (std::size_t i = 0; i < 4; ++i)
      c[i] = _mm_load_ps (tf.col (i).data ());
  }

  void so3 (const float* src, float* tgt) const
  {
    __m128 p0 = _mm_mul_ps (_mm_load_ps1 (&src[0]), c[0]);
    __m128 p1 = _mm_mul_ps (_mm_load_ps1 (&src[1]), c[1]);
    __m128 p2 = _mm_mul_ps (_mm_load_ps1 (&src[2]), c[2]);
    _mm_store_ps (tgt, _mm_add_ps(p0, _mm_add_ps(p1, p2)));
  }

  void se3 (const float* src, float* tgt) const
  {
    __m128 p0 = _mm_mul_ps (_mm_load_ps1 (&src[0]), c[0]);
    __m128 p1 = _mm_mul_ps (_mm_load_ps1 (&src[1]), c[1]);
    __m128 p2 = _mm_mul_ps (_mm_load_ps1 (&src[2]), c[2]);
    _mm_store_ps (tgt, _mm_add_ps(p0, _mm_add_ps(p1, _mm_add_ps(p2, c[3]))));
  }
};

#if !defined(__AVX__)

/** Optimized version for double-precision transform using SSE2 intrinsics. */
template<>
struct Transformer<double>
{
  /// Columns of the transform matrix stored in XMM registers.
  __m128d c[4][2];

  Transformer(const Eigen::Matrix4d& tf)
  {
    for (std::size_t i = 0; i < 4; ++i)
    {
      c[i][0] = _mm_load_pd (tf.col (i).data () + 0);
      c[i][1] = _mm_load_pd (tf.col (i).data () + 2);
    }
  }

  void so3 (const float* src, float* tgt) const
  {
    __m128d xx = _mm_cvtps_pd (_mm_load_ps1 (&src[0]));
    __m128d p0 = _mm_mul_pd (xx, c[0][0]);
    __m128d p1 = _mm_mul_pd (xx, c[0][1]);

    for (std::size_t i = 1; i < 3; ++i)
    {
      __m128d vv = _mm_cvtps_pd (_mm_load_ps1 (&src[i]));
      p0 = _mm_add_pd (_mm_mul_pd (vv, c[i][0]), p0);
      p1 = _mm_add_pd (_mm_mul_pd (vv, c[i][1]), p1);
    }

    _mm_store_ps (tgt, _mm_movelh_ps (_mm_cvtpd_ps (p0), _mm_cvtpd_ps (p1)));
  }

  void se3 (const float* src, float* tgt) const
  {
    __m128d p0 = c[3][0];
    __m128d p1 = c[3][1];

    for (std::size_t i = 0; i < 3; ++i)
    {
      __m128d vv = _mm_cvtps_pd (_mm_load_ps1 (&src[i]));
      p0 = _mm_add_pd (_mm_mul_pd (vv, c[i][0]), p0);
      p1 = _mm_add_pd (_mm_mul_pd (vv, c[i][1]), p1);
    }

    _mm_store_ps (tgt, _mm_movelh_ps (_mm_cvtpd_ps (p0), _mm_cvtpd_ps (p1)));
  }
};

#else

/** Optimized version for double-precision transform using AVX intrinsics. */
template<>
struct Transformer<double>
{
  __m256d c[4];

  Transformer(const Eigen::Matrix4d& tf)
  {
    for (std::size_t i = 0; i < 4; ++i)
      c[i] = _mm256_load_pd (tf.col (i).data ());
  }

  void so3 (const float* src, float* tgt) const
  {
    __m256d p0 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[0])), c[0]);
    __m256d p1 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[1])), c[1]);
    __m256d p2 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[2])), c[2]);
    _mm_store_ps (tgt, _mm256_cvtpd_ps (_mm256_add_pd(p0, _mm256_add_pd(p1, p2))));
  }

  void se3 (const float* src, float* tgt) const
  {
    __m256d p0 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[0])), c[0]);
    __m256d p1 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[1])), c[1]);
    __m256d p2 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[2])), c[2]);
    _mm_store_ps (tgt, _mm256_cvtpd_ps (_mm256_add_pd(p0, _mm256_add_pd(p1, _mm256_add_pd(p2, c[3])))));
  }
};

#endif // !defined(__AVX__)
#endif // defined(__SSE2__)

template <typename PointT, typename Scalar> void
transformPointCloudLocal (const pcl::PointCloud<PointT> &cloud_in,
                     pcl::PointCloud<PointT> &cloud_out,
                     const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                     bool copy_all_fields)
{
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header   = cloud_in.header;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.points.reserve (cloud_in.points.size ());
    if (copy_all_fields)
      cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
    else
      cloud_out.points.resize (cloud_in.points.size ());
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
  }

  Transformer<Scalar> tf (transform.matrix ());
  if (cloud_in.is_dense)
  {
    // If the dataset is dense, simply transform it!
    for (std::size_t i = 0; i < cloud_out.points.size (); ++i)
      tf.se3 (cloud_in[i].data, cloud_out[i].data);
  }
  else
  {
    // Dataset might contain NaNs and Infs, so check for them first,
    // otherwise we get errors during the multiplication (?)
    for (std::size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      if (!std::isfinite (cloud_in.points[i].x) ||
          !std::isfinite (cloud_in.points[i].y) ||
          !std::isfinite (cloud_in.points[i].z))
        continue;
      tf.se3 (cloud_in[i].data, cloud_out[i].data);
    }
  }
}

/*****************************************************************/
//Color Definitions
/****************************************************************/
#define _NORMAL_    "\x1b[0m"
#define _BLACK_     "\x1b[30;47m"
#define _RED_       "\x1b[31;40m"
#define _GREEN_     "\x1b[32;40m"
#define _YELLOW_    "\x1b[33;40m"
#define _BLUE_      "\x1b[34;40m"
#define _MAGENTA_   "\x1b[35;40m"
#define _CYAN_      "\x1b[36;40m"
#define _WHITE_     "\x1b[37;40m"
#define _BRED_      "\x1b[1;31;40m"
#define _BGREEN_    "\x1b[1;32;40m"
#define _BYELLOW_   "\x1b[1;33;40m"
#define _BBLUE_     "\x1b[1;34;40m"
#define _BMAGENTA_  "\x1b[1;35;40m"
#define _BCYAN_     "\x1b[1;36;40m"
#define _BWHITE_    "\x1b[1;37;40m"
/******************************************************************/
//Print Utilities for debugging.
/*******************************************************************/
#define dbg_color(...) do { printf(__VA_ARGS__); } while(0)

#define tic() std::chrono::high_resolution_clock::time_point END_TIME,START_TIME=std::chrono::high_resolution_clock::now()

#define reset() START_TIME=std::chrono::high_resolution_clock::now()

#define toc()                                                   \
do                                                              \
{                                                               \
  dbg_color(_GREEN_);                                         \
    END_TIME=std::chrono::high_resolution_clock::now();         \
    std::cout << " Time elapsed in "<<__func__<<"() : "         \
    << std::chrono::duration_cast<std::chrono::microseconds>(END_TIME - START_TIME).count() << " microseconds.\n"; \
    dbg_color(_NORMAL_);                                        \
}while(0)


using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

template <typename PointT> static void visualizePointCloud(const pcl::PointCloud<PointT>& cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<PointT> (cloud.makeShared(), "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	// viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);

	}
}

void print(){std::cout<<std::endl;}
	template<typename T,typename... Args>
void print(T Contents, Args... args) 
{
	std::cout<< (Contents) <<" ";print(args...);
}


void pclXYZRGBNormalToPly(const std::string filename,pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud)
{
	ofstream f(filename);
	string header="property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n";
	f<<"ply\nformat ascii 1.0\n";
	f<<"element vertex ";
	f<<(to_string(cloud.points.size())+"\n");
	f<<header;
	for(auto pt:cloud.points)
	{
		f<<pt.x<<" "<<pt.y<<" "<<pt.z<<" "<<static_cast<int>(pt.r)<<" "<<static_cast<int>(pt.g)<<" "<<static_cast<int>(pt.b)<<" "<<pt.normal[0]<<" "<<pt.normal[1]<<" "<<pt.normal[2]<<"\n";
	}
	f.close();
}

inline float* apply_transformation_optimized(float* data,Eigen::Affine3d& transformation)
{
	Transformer<double> tf(transformation.matrix());
	float output[3];
	tf.se3(data,output);
	return output;
}
inline void apply_transformation_optimized(float* data,float* output,Eigen::Affine3d& transformation)
{
	Transformer<double> tf(transformation.matrix());
	tf.se3(data,output);
}

inline Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat)
{
	Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
	Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
	data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
	data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
	Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
	Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
	transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
	return transformed_data_mat.transpose();
}

pcl::PointCloud<pcl::PointXYZRGB> transformPointCloudParallel(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud,Eigen::MatrixXd& a_T_b)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	#pragma omp parallel for
	for(int i=0;i<point_cloud.points.size();i++)
	{
		auto point=point_cloud.points[i];
		Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
		pts(0,0)=point.x;
		pts(0,1)=point.y;
		pts(0,2)=point.z;
		Eigen::MatrixXd pts_trans=apply_transformation(pts,a_T_b);
		pcl::PointXYZRGB pt;
		pt.x=pts_trans(0,0);
		pt.y=pts_trans(0,1);
		pt.z=pts_trans(0,2);
		pt.rgb=point.rgb;
		point_cloud.points[i]=pt;
	}
	return point_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB> transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud,Eigen::Affine3d& a_T_b)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	for(int i=0;i<point_cloud.points.size();i++)
	{
		auto point=point_cloud.points[i];
		float output[3];
		std::cout<<sizeof(point.data)/sizeof(float)<<std::endl;
		apply_transformation_optimized(point.data,output,a_T_b);
		pcl::PointXYZRGB pt;
		pt.data[0]=output[0];
		pt.data[1]=output[1];
		pt.data[2]=output[2];
		pt.rgb=point.rgb;
		point_cloud.points[i]=pt;
	}
	return point_cloud;
}


int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PLYReader Reader;
	Reader.read("/home/rex/Downloads/after_gaussian(1).ply", *cloud_ptr);
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	for(auto pt:cloud_ptr->points)
		cloud.points.push_back(pt);
	Eigen::MatrixXd transformation=Eigen::MatrixXd::Identity(4,4);
	Eigen::Affine3d temp = Eigen::Affine3d::Identity();
	tic();
	pcl::PointCloud<pcl::PointXYZRGB> cloud_transformed=transformPointCloud(cloud,temp);
	toc();
	visualizePointCloud<pcl::PointXYZRGB>(cloud_transformed);
	reset();
	cloud_transformed=transformPointCloudParallel(cloud,transformation);
	cout<<cloud.is_dense<<endl;
	toc();
	reset();
	pcl::transformPointCloud (cloud,cloud,temp);
	toc();
	reset();
	transformPointCloudLocal<pcl::PointXYZRGB,double>(cloud,cloud,temp,true);
	toc();
	pcl::PointCloud<pcl::PointXYZRGB> exp;
	temp(0,3)=1;
	pcl::PointXYZRGB temp_point;
	temp_point.x=0;
	temp_point.y=0;
	temp_point.z=0;
	temp_point.r=255;
	temp_point.g=0;
	temp_point.b=0;
	exp.points.push_back(temp_point);
	for(auto pt:exp.points)
	print(pt.x,pt.y,pt.z);
	pcl::PointCloud<pcl::PointXYZRGB> exp_t = transformPointCloud(exp,temp);
	for(auto pt:exp_t.points)
	print(pt.x,pt.y,pt.z);
	return 0;
}
