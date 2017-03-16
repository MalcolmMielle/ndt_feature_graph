#include <iostream>
#include <boost/program_options.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_registration/ndt_matcher_d2d_feature.h>
#include <ndt_feature/ndt_matcher_d2d_fusion.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/cell_vector.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/pointcloud_utils.h>

#include <ndt_feature/ndt_rviz.h>
#include <ndt_feature/utils.h>

#include <gnuplot-iostream.h> // sudo apt-get install libgnuplot-iostream-dev

using namespace std;
using namespace lslgeneric;
namespace po = boost::program_options;

#define POLY
#ifdef POLY
double compute_score(double x, double x0, double c1, double c2) {
double X = x - x0;
return X*X;
}

double compute_gradient(double x, double x0, double c1, double c2) {
double X = x - x0;
return 2*X;
}

double compute_hessian(double x, double x0, double c1, double c2) {
double X = x - x0;
return 2;
}
#else

double compute_score(double x, double x0, double c1, double c2) {
  double X = x - x0;
  return -exp(-X*X);
}

double compute_gradient(double x, double x0, double c1, double c2) {
  double X = x - x0;
  return 2*X*exp(-X*X);
}

double compute_hessian(double x, double x0, double c1, double c2) {
  double X = x - x0;
  return -4*X*X*exp(-X*X) + 2* exp(-X*X);
}
#endif

double compute_score2d(const Eigen::Vector2d &x,
                       const Eigen::Vector2d &x0,
                       const Eigen::Matrix2d &C) {
  Eigen::Vector2d X = x - x0;
  return X.transpose()*C*X;
}

Eigen::Vector2d compute_gradient2d(const Eigen::Vector2d &x,
                                   const Eigen::Vector2d &x0,
                                   const Eigen::Matrix2d &C) {
  Eigen::Vector2d X = x - x0;
  return Eigen::Vector2d(2*X(0)*C(0,0) + C(0,1)*X(1) + C(1,0)*X(1),
                         X(0)*C(0,1) + C(1,0)*X(0) + 2*X(1)*C(1,1));
}

Eigen::Matrix2d compute_hessian2d(const Eigen::Vector2d &x,
                                  const Eigen::Vector2d &x0,
                                  const Eigen::Matrix2d &C) {
  Eigen::Matrix2d H;
  H << 2*C(0,0), C(0,1)+C(1,0), C(0,1)+C(1,0), 2*C(1,1);
  return H;
}

double compute_score6d(const Eigen::Matrix<double,6,1> &x,
                       const Eigen::Matrix<double,6,1> &x0,
                       const Eigen::Matrix<double,6,6> &C) {
  Eigen::Matrix<double,6,1> X = x - x0;
  return X.transpose()*C*X;
}
     
Eigen::Matrix<double,6,6> compute_hessian6d(const Eigen::Matrix<double,6,1> &x,
                                            const Eigen::Matrix<double,6,1> &x0,
                                            const Eigen::Matrix<double,6,6> &C) {
  Eigen::Matrix<double,6,6> H;
  H << C(0,0)+C(0,0), C(1,0)+C(0,1), C(2,0)+C(0,2), C(3,0)+C(0,3), C(4,0)+C(0,4), C(5,0)+C(0,5),
       C(0,1)+C(1,0), C(1,1)+C(1,1), C(2,1)+C(1,2), C(3,1)+C(1,3), C(4,1)+C(1,4), C(5,1)+C(1,5),
       C(0,2)+C(2,0), C(1,2)+C(2,1), C(2,2)+C(2,2), C(3,2)+C(2,3), C(4,2)+C(2,4), C(5,2)+C(2,5),
       C(0,3)+C(3,0), C(1,3)+C(3,1), C(2,3)+C(3,2), C(3,3)+C(3,3), C(4,3)+C(3,4), C(5,3)+C(3,5),
       C(0,4)+C(4,0), C(1,4)+C(4,1), C(2,4)+C(4,2), C(3,4)+C(4,3), C(4,4)+C(4,4), C(5,4)+C(4,5),
       C(0,5)+C(5,0), C(1,5)+C(5,1), C(2,5)+C(5,2), C(3,5)+C(5,3), C(4,5)+C(5,4), C(5,5)+C(5,5);
       
  return H;
}

Eigen::Matrix<double,6,1> compute_gradient6d(const Eigen::Matrix<double,6,1> &x,
                                             const Eigen::Matrix<double,6,1> &x0,
                                             const Eigen::Matrix<double,6,6> &C) {
  Eigen::Matrix<double,6,1> X = x - x0;
  return compute_hessian6d(x, x0, C)*X;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_hessian_test");
    Gnuplot gp;

    po::options_description desc("Allowed options");
    Eigen::Matrix<double,6,1> pose_v, init_pose_v;
    Eigen::Matrix<double,6,6> cov;
    double step_size;
    double term_crit;
    int max_iter;

    desc.add_options()
    ("help", "produce help message")
    ("x", po::value<double>(&pose_v(0))->default_value(0.), "x pos gt offset")
    ("y", po::value<double>(&pose_v(1))->default_value(0.), "y pos gt offset")
    ("Z", po::value<double>(&pose_v(5))->default_value(0.), "z axis rot gt offset")
    ("init_x", po::value<double>(&init_pose_v(0))->default_value(1.), "x pos odom offset")
    ("init_y", po::value<double>(&init_pose_v(1))->default_value(0.), "y pos odom offset")
    ("init_Z", po::value<double>(&init_pose_v(5))->default_value(0.), "z axis rot odom offset")
    ("step_size", po::value<double>(&step_size)->default_value(1), "step size")
    ("term_crit", po::value<double>(&term_crit)->default_value(0.00001), "termination critera in increment length")
    ("max_iter", po::value<int>(&max_iter)->default_value(100), "maximum number of iterations")      ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }


    std::cout << "init_pose_v : " << init_pose_v << std::endl;
    std::cout << "pose_v : " << pose_v << std::endl;
    

    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    {
      //
      const double c1 = 1;
      const double c2 = 1;
      
      double x = init_pose_v(0);
      double x0 = pose_v(0);
      
      double score = compute_score(x, x0, c1, c2);
      double gradient = compute_gradient(x, x0, c1, c2);
      double hessian = compute_hessian(x, x0, c1, c2);
      double dx = -gradient/hessian*step_size;
 
      std::cout << "x0 : " << x0 << std::endl;
      std::cout << "dx : " << dx << std::endl;
      std::cout << "x : " << x << std::endl;
      std::cout << "score : " << score << std::endl;
      std::cout << "gradient: " << gradient << std::endl;
      std::cout << "hessian : " << hessian << std::endl;
      
      std::vector<std::pair<double, double> > xy_pts;
      xy_pts.push_back(std::make_pair(x, score));
      int i = 0;    
      while (fabs(dx) > term_crit && i < max_iter) {
        
        x += dx;
        
        score = compute_score(x, x0, c1, c2);
        gradient = compute_gradient(x, x0, c1, c2);
        hessian = compute_hessian(x, x0, c1, c2);
        
        std::cout << "=== iter : " << i << " ===" << std::endl;
        std::cout << "x0 : " << x0 << std::endl;
        std::cout << "dx : " << dx << std::endl;
        std::cout << "x : " << x << std::endl;
        std::cout << "score : " << score << std::endl;
        std::cout << "gradient: " << gradient << std::endl;
        std::cout << "hessian : " << hessian << std::endl;
        
        xy_pts.push_back(std::make_pair(x, score));
        
        dx = -gradient/hessian*step_size;
        i++;
      }
      
      std::vector<std::pair<double, double> > xy_pts_A, xy_pts_B, xy_pts_C;
      for(x=-3; x<3; x+=0.01) {
        double yA = compute_score(x, x0, c1, c2);
        double yB = compute_gradient(x, x0, c1, c2);
        double yC = compute_hessian(x, x0, c1, c2);
        xy_pts_A.push_back(std::make_pair(x, yA));
        xy_pts_B.push_back(std::make_pair(x, yB));
        xy_pts_C.push_back(std::make_pair(x, yC));
      }
      
      gp << "set xrange [-3:3]\nset yrange [-2:2]\n";
      gp << "plot '-' with lines title 'score', '-' with lines title 'gradient', '-' with lines title 'hessian', '-' with circles title 'newton steps'\n";
      gp.send1d(xy_pts_A);
      gp.send1d(xy_pts_B);
      gp.send1d(xy_pts_C);
      gp.send1d(xy_pts);
      
    }
    
    std::cout << " -------------- 2D ---------------- " << std::endl;

    {

      Eigen::Vector2d x(init_pose_v(0), init_pose_v(1));
      Eigen::Vector2d x0(pose_v(0), pose_v(1));
      Eigen::Matrix2d cov;
      cov << 0.1,0,0,1.0;
      std::cout << "cov : " << cov << std::endl;
      Eigen::Matrix2d C = cov.inverse();
      std::cout << "C : " << C << std::endl;

      double score = compute_score2d(x, x0, C);
      Eigen::Vector2d gradient = compute_gradient2d(x, x0, C);
      Eigen::Matrix2d hessian = compute_hessian2d(x, x0, C);
      Eigen::Vector2d dx;
      dx = -hessian.ldlt().solve(gradient);
 
      std::cout << "This should be equal to the gradient(!)" << std::endl;
      std::cout << "hessian*dx : " << hessian*dx << std::endl;

      std::cout << "x0 : " << x0 << std::endl;
      std::cout << "dx : " << dx << std::endl;
      std::cout << "x : " << x << std::endl;
      std::cout << "score : " << score << std::endl;
      std::cout << "gradient: " << gradient << std::endl;
      std::cout << "hessian : " << hessian << std::endl;
      
      int i = 0;    
      while (dx.norm() > term_crit && i < max_iter) {
        
        x += dx;
        
        score = compute_score2d(x, x0, C);
        gradient = compute_gradient2d(x, x0, C);
        hessian = compute_hessian2d(x, x0, C);
        
        std::cout << "=== iter : " << i << " ===" << std::endl;
        std::cout << "x0 : " << x0 << std::endl;
        std::cout << "dx : " << dx << std::endl;
        std::cout << "x : " << x << std::endl;
        std::cout << "score : " << score << std::endl;
        std::cout << "gradient: " << gradient << std::endl;
        std::cout << "hessian : " << hessian << std::endl;
        
        dx = -hessian.ldlt().solve(gradient)*step_size;
        i++;
      }


      // 
      
    }

    std::cout << " -------------- 6D ---------------- " << std::endl;

    {

      Eigen::Matrix<double,6,1> x;
      x << init_pose_v(0), init_pose_v(1), 2., 3., 4., 5.;
      Eigen::Matrix<double,6,1> x0;
      x0 << pose_v(0), pose_v(1), 3., 4., 5., 6.;
      Eigen::Matrix<double,6,6> cov;
      cov << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.2, 0.1,
        0.0, 0.0, 0.0, 0.0, 0.1, 0.4;
      
      std::cout << "cov : " << cov << std::endl;
      Eigen::Matrix<double,6,6> C = cov.inverse();
      std::cout << "C : " << C << std::endl;

      double score = compute_score6d(x, x0, C);
      Eigen::Matrix<double,6,1> gradient = compute_gradient6d(x, x0, C);
      Eigen::Matrix<double,6,6> hessian = compute_hessian6d(x, x0, C);
      Eigen::Matrix<double,6,1> dx;
      dx = -hessian.ldlt().solve(gradient);
 
      std::cout << "This should be equal to the gradient(!)" << std::endl;
      std::cout << "hessian*dx : " << hessian*dx << std::endl;

      std::cout << "x0 : " << x0 << std::endl;
      std::cout << "dx : " << dx << std::endl;
      std::cout << "x : " << x << std::endl;
      std::cout << "score : " << score << std::endl;
      std::cout << "gradient: " << gradient << std::endl;
      std::cout << "hessian : " << hessian << std::endl;
      
      int i = 0;    
      while (dx.norm() > term_crit && i < max_iter) {
        
        x += dx;
        
        score = compute_score6d(x, x0, C);
        gradient = compute_gradient6d(x, x0, C);
        hessian = compute_hessian6d(x, x0, C);
        
        std::cout << "=== iter : " << i << " ===" << std::endl;
        std::cout << "x0 : " << x0 << std::endl;
        std::cout << "dx : " << dx << std::endl;
        std::cout << "x : " << x << std::endl;
        std::cout << "score : " << score << std::endl;
        std::cout << "gradient: " << gradient << std::endl;
        std::cout << "hessian : " << hessian << std::endl;
        
        dx = -hessian.ldlt().solve(gradient)*step_size;
        i++;
      }


      // 
      
    }
    

}
