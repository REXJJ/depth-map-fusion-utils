#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <omp.h>

#if defined(__SSE2__)
#include <xmmintrin.h>
#endif

#if defined(__AVX__)
#include <immintrin.h>
#endif

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

inline void apply_transformation_optimized(float* data,float* output,Eigen::Affine3d& transformation)
{
	Transformer<double> tf(transformation.matrix());
	tf.se3(data,output);
}

int main(int argc, char** argv)
{

	float input[4]={0,0,0,0};
	float output[4];
	Eigen::Affine3d transformation=Eigen::Affine3d::Identity();
	transformation(1,3)=3;
	apply_transformation_optimized(input,output,transformation);
	for(int i=0;i<3;i++)
		std::cout<<output[i]<<" ";
	std::cout<<std::endl;
	return 0;
}
