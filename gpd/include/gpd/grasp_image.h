/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Andreas ten Pas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef GRASP_IMAGE_H
#define GRASP_IMAGE_H

#include <iostream>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


/** GraspImage class
 *
 * \brief Create grasp images.
 *
 * This class contains a set of methods to create grasp images. A grasp image is a representation of a grasp candidate
 * that can be used as input for a convolutional neural network (or another classification method).
 *
*/
class GraspImage
{
public:

  /**
   * \brief Constructor.
   * \param image_size the size of the image
   * \param num_channels the number of channels of the image
   * \param is_plotting if the image is visualized
   */
  GraspImage(int image_size, int num_channels, bool is_plotting);

  /**
   * \brief Virtual destructor.
   */
  virtual ~GraspImage() { };

  /**
   * \brief Virtual function to calculate a grasp image.
   */
  virtual cv::Mat calculateImage() = 0;

  /**
   * \brief Find the indices of the pixels that are occupied by a given list of points.
   * \param points the points
   * \return the indices occupied by the points
   */
  static Eigen::VectorXi findCellIndices(const Eigen::Matrix3Xd& points);

  /**
   * \brief Create a binary image based on which pixels are occupied by the points.
   * \param cell_indices the indices of the points in the image
   * \return the image
   */
  static cv::Mat createBinaryImage(const Eigen::VectorXi& cell_indices);

  /**
   * \brief Create an RGB image based on the surface normals of the points.
   * \param normals the surface normals
   * \param cell_indices the indices of the points in the image
   * \return the image
   */
  static cv::Mat createNormalsImage(const Eigen::Matrix3Xd& normals, const Eigen::VectorXi& cell_indices);

  /**
   * \brief Create a grey value image based on the depth value of the points.
   * \param points the points
   * \param cell_indices the indices of the points in the image
   * \return the image
   */
  static cv::Mat createDepthImage(const Eigen::Matrix3Xd& points, const Eigen::VectorXi& cell_indices);

  /**
   * \brief Create a grey value image based on the depth of the shadow points.
   * \param points the shadow points
   * \param cell_indices the indices of the shadow points in the image
   * \return the image
   */
  static cv::Mat createShadowImage(const Eigen::Matrix3Xd& points, const Eigen::VectorXi& cell_indices);

  /**
   * \brief Set the size of the image.
   * \param image_size the image size
   */
  static void setImageSize(int image_size)
  {
    image_size_ = image_size;
  }


protected:

  int num_channels_; ///< the number of channels of the grasp image
  bool is_plotting_; ///< if the grasp image is visualized

  static int image_size_; ///< the size of the grasp image (height x width)


private:

  /**
   * \brief Round a floating point vector to the closest, smaller integers.
   * \param a the floating point vector
   * \return the vector containing the integers
   */
  static Eigen::VectorXi floorVector(const Eigen::VectorXd& a);
};


# endif /* GRASP_IMAGE_H */
