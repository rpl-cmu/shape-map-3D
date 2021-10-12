/**
   * @file gpfactor.h
   * @brief Wrapper to build cpp header with MATLAB 
   * @author Sudharshan Suresh 
   * @date May 2021

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

class gtsam::Vector2; class gtsam::Vector3;
class gtsam::Rot2;    class gtsam::Rot3;
class gtsam::Pose2;   class gtsam::Pose3;

class gtsam::Matrix3; class gtsam::Matrix4;

class gtsam::GaussianFactorGraph; class gtsam::Values;

virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NonlinearFactorGraph;
virtual class gtsam::NumericalDerivative;
virtual class gtsam::NoiseModelFactor;

namespace gpfactor {
  #include <gpfactor/gp/GaussianProcessSpatialPrior.h>

  // 2-D and 3-D GP-SG
  template <DOF = {2, 3}>
  virtual class GaussianProcessSpatialPrior : gtsam::NoiseModelFactor {
    GaussianProcessSpatialPrior(size_t yKey, const Matrix& xm, const Matrix& xq,
      const Vector& ym,  const Vector& theta, const Vector& measurementNoise);
  };
}

