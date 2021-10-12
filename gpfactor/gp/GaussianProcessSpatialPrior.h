/**
   * @file GaussianProcessSpatialPrior.h
   * @brief Unary Gaussian potential for surface measurements in spatial graph
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

#pragma once

#include <boost/lexical_cast.hpp>
#include <boost/optional/optional_io.hpp>
#include <Eigen/Dense>
#include <gtsam/geometry/concepts.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

namespace gpfactor {

/** Spatial GP prior (2D or 3D) */
template <int Dim>
class GaussianProcessSpatialPrior: public gtsam::NoiseModelFactor1<Eigen::Matrix<double, Dim + 1, 1> > {

private:

  typedef Eigen::Matrix<double, Dim + 1, 1> YDim;
  typedef GaussianProcessSpatialPrior<Dim> This;
  typedef gtsam::NoiseModelFactor1<YDim> Base;
  double kernType_; // 0 - thinplate, 1 - RBF
  Eigen::Vector2d kernParams_; 
  YDim ypred_;

public:

  // constructor and destructor
  GaussianProcessSpatialPrior() {}
  GaussianProcessSpatialPrior(gtsam::Key yKey, const gtsam::Matrix& xq, const gtsam::Matrix& xm,
         const gtsam::Vector& ym, const gtsam::Vector& theta, const gtsam::Vector& measurementNoise) :
        Base(gtsam::noiseModel::Gaussian::Covariance(getMeanAndReturnVariance(xq, xm, ym, theta, measurementNoise)), yKey) {}
  virtual ~GaussianProcessSpatialPrior() {}

  // Deep copy
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /// Error function
  gtsam::Vector evaluateError(const YDim& y, boost::optional<gtsam::Matrix&> H1 = boost::none) const {
    auto err = [&] (const YDim& yq)
    {   
        return (ypred_ - yq);
    };
    // Jacobians
    // if (H1)  *H1 = gtsam::numericalDerivative11<Vector, YDim>(err, y);
    if (H1) *H1 = -Eigen::Matrix<double, Dim + 1, Dim + 1>::Identity();
    return err(y);
  }
    size_t dim() const { return Dim; }

    // print
    void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
      std::cout << s << "Gaussian Process Spatial Prior<" << Dim << ">" << std::endl;
      Base::print("", keyFormatter);
    }

private:
  // get GP mean and covariance from measurements 
  gtsam::Matrix getMeanAndReturnVariance(const gtsam::Matrix& xq, const gtsam::Matrix& xm, const gtsam::Vector& ym, const gtsam::Vector& theta, const gtsam::Vector& measurementNoise)  {
    kernParams_ << theta(1), theta(2); 
    kernType_ = theta(0);
    gtsam::Matrix mN = measurementNoise.asDiagonal();
    gtsam::Matrix Kmm = kernel(xm, xm) + mN; 
    gtsam::Matrix Kqm =  kernel(xq, xm);  
    gtsam::Matrix Kqq =  kernel(xq, xq);  

    gtsam::Matrix Kmm_L = Eigen::LLT<Eigen::MatrixXd>(Kmm).matrixL(); // cholesky with PSD
    gtsam::Matrix x = Kmm_L.triangularView<Eigen::Lower>().solve(ym);

    // compute mean
    gtsam::Matrix Kmm_alpha = (Kmm_L.triangularView<Eigen::Lower>().transpose()).solve(x);
    ypred_ =  Kqm * Kmm_alpha;

    // compute variance
    gtsam::Matrix V = Kmm_L.triangularView<Eigen::Lower>().solve(Kqm.transpose());
    gtsam::Matrix C = Kqq - V.transpose()*V;
    return C;
  }

  // Get the Kxstarxstar kernel (3, 3)
  gtsam::Matrix kernel(const gtsam::Matrix& x1, const gtsam::Matrix& x2) const{
      int n = static_cast<int>(x1.rows());
      int m = static_cast<int>(x2.rows());
      Eigen::MatrixXd K =  Eigen::MatrixXd::Zero((Dim + 1)*n,(Dim + 1)*m);
      for (int i = 0; i < n; i++) {
          for (int j = 0; j < m; j++) {
              if (kernType_ == 0) K.block<(Dim + 1),(Dim + 1)>((Dim + 1)*i, (Dim + 1)*j) = kernelBlockThinPlate(x1.row(i), x2.row(j));
              else if (kernType_ == 1)  K.block<(Dim + 1),(Dim + 1)>((Dim + 1)*i, (Dim + 1)*j) = kernelBlockGaussian(x1.row(i), x2.row(j));
          }
      }
      return K;
  }

  // individual kernel elements: thinplate
  gtsam::Matrix kernelBlockThinPlate(const gtsam::Matrix& x1, const gtsam::Matrix& x2) const {
        Eigen::MatrixXd block =  Eigen::MatrixXd::Zero(Dim + 1, Dim + 1);
        double xyNorm = Norm(x1, x2);
        double R = kernParams_(0);
        double sigma = kernParams_(1);

        if (xyNorm > 1e-4) {
            // x1 != x2
            block(0,0) = 2.0 * std::pow(xyNorm, 3) - 3.0 * R * std::pow(xyNorm, 2) + std::pow(R, 3); //  cov(di, dj) 
            block(0,1) = 2.0 * 3.0 * xyNorm * (-x1(0) + x2(0)) - 3.0 * R * 2.0 * (-x1(0) + x2(0)); // cov(di, w1j)
            block(0,2) = 2.0 * 3.0 * xyNorm * (-x1(1) + x2(1)) - 3.0 * R * 2.0 * (-x1(1) + x2(1)); // cov(di, w2j)

            block(1,0) = -block(0,1); // cov(w1i, dj)
            block(1,1) = 2.0 * 3.0 * ((-x1(0) + x2(0))/xyNorm) * (x1(0) - x2(0)) + 2.0 * 3.0 * xyNorm * (-1.0) - 3.0 * R * 2.0 * (-1.0); // cov(w1i, w1j)
            block(1,2) = 2.0 * 3.0 * ((-x1(1) + x2(1))/xyNorm) * (x1(0) - x2(0)); // cov(w1i, w2j)

            block(2,0) = -block(0,2); // cov(w2i, dj)
            block(2,1) =  block(1,2); // cov(w2i, w1j)
            block(2,2) = 2.0 * 3.0 * ((-x1(1) + x2(1))/xyNorm) * (x1(1) - x2(1)) + 2.0 * 3.0 * xyNorm * (-1.0) - 3.0 * R * 2.0 * (-1.0); // cov(w2i, w2j)

            // 3d data
            if (Dim == 3) {
                block(0,3) = 2.0 * 3.0 * xyNorm * (-x1(2) + x2(2)) - 3.0 * R * 2.0 * (-x1(2) + x2(2)); // cov(di, w3j)
                block(1,3) = 2.0 * 3.0 * ((-x1(2) + x2(2))/xyNorm) * (x1(0) - x2(0)); // cov(w1i, w3j)
                block(2,3) = 2.0 * 3.0 * ((-x1(2) + x2(2))/xyNorm) * (x1(1) - x2(1)); // cov(w2i, w3j)

                block(3,0) =  -block(0,3); // cov(w3i, dj)
                block(3,1) = block(1,3); // cov(w3i, w1j)
                block(3,2) = block(2,3); // cov(w3i, w2j)
                block(3,3) = 2.0 * 3.0 * ((-x1(2) + x2(2))/xyNorm) * (x1(2) - x2(2)) + 2.0 * 3.0 * xyNorm * (-1.0) - 3.0 * R * 2.0 * (-1.0); // cov(w3i, w3j)
            }
        } else {
            // x1 == x2 diag
            block(0,0) = 2.0 * std::pow(xyNorm, 3) - 3.0 * R * std::pow(xyNorm, 2) + std::pow(R, 3); 
            block(1,1) = 6.0 * R; 
            block(2,2) = 6.0 * R; 
            if (Dim == 3) {
              block(3,3) = 6.0 * R; 
            }
        }
        return std::pow(sigma, 2) * block;
    }

  // individual kernel elements: SE kernel (BUGGY, not tested)
  gtsam::Matrix kernelBlockGaussian(const gtsam::Matrix& x1, const gtsam::Matrix& x2) const{
      Eigen::MatrixXd block =  Eigen::MatrixXd::Zero(3,3);
      double l2_inv = 1.0 / std::pow(kernParams_(0), 2);
      double sigma = kernParams_(1);

      double diffX = x1(0) - x2(0);
      double diffY = x1(1) - x2(1);

      block(0,0) = 1.0;

      block(1,1) = l2_inv * (1 - std::pow(diffX,2) * l2_inv);
      block(2,2) = l2_inv * (1 - std::pow(diffY,2) * l2_inv);
      block(1,2) = -std::pow(l2_inv,2) * diffX * diffY;

      block(0,1) = l2_inv * diffX;
      block(0,2) = l2_inv * diffY;

      block(1,0) = -block(0,1);
      block(2,0) = -block(0,2);
      block(2,1) = block(1,2);

      block *= std::pow(sigma, 2) * std::exp(-0.5 * Norm(x1, x2) * l2_inv);
      return block;
  }

    // Euclidean distance norm Norm(x - y)
    double Norm(const gtsam::Matrix& x1, const gtsam::Matrix& x2) const {
        if (Dim == 3) {
          return std::sqrt(std::pow(x1(0) - x2(0), 2) + std::pow(x1(1) - x2(1), 2) + std::pow(x1(2) - x2(2), 2)); 
        } else {
          return std::sqrt(std::pow(x1(0) - x2(0), 2) + std::pow(x1(1) - x2(1), 2)); 
        }
    }

  // serialization
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(kernType_);
    ar & BOOST_SERIALIZATION_NVP(kernParams_);
    ar & BOOST_SERIALIZATION_NVP(ypred_);
  }

}; // GaussianProcessSpatialPrior

} // namespace gpfactor

/// traits
namespace gtsam {
template<int Dim>
struct traits<gpfactor::GaussianProcessSpatialPrior<Dim> > : public Testable<
    gpfactor::GaussianProcessSpatialPrior<Dim> > {};
}
