#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>
#include <boost/foreach.hpp>

#include <fstream>
#include <iostream>
#include <ctime>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp> // for std::pair

using namespace std;
typedef vector<InterestPoint*> InterestPointVec;

void generateReadings(std::vector<double> &phi, std::vector<double> &rho) {
  
  int nb_readings = 361;
  phi.resize(nb_readings);
  rho.resize(nb_readings);
  double start = -M_PI/2.;
  double width = M_PI;
  double incr = M_PI/(1.*nb_readings);
  for (int i = 0; i < nb_readings; i++) {
    phi[i] = start + i *incr;
    rho[i] = 4+2*sin(i/20.)+0.5*cos(i/3.); // Magic empirical formula...
    std::cout << "rho : " << rho[i] << std::endl;
  }
}

SimpleMinMaxPeakFinder* createPeakFinder ()
{
  return new SimpleMinMaxPeakFinder(0.34, 0.001);
}

Detector* createDetector (SimpleMinMaxPeakFinder* peak_finder)
{
  const double scale = 5.0;
  const double dmst = 2.0;
  const double base_sigma = 0.2;
  const double sigma_step = 1.4;
  CurvatureDetector* det = new CurvatureDetector(peak_finder, scale, base_sigma,
                                                 sigma_step, dmst);
  det->setUseMaxRange(false);
  return det;
}

DescriptorGenerator* createDescriptor (HistogramDistance<double>* dist)
{
  const double min_rho = 0.02;
  const double max_rho = 1.;//0.5;
  const double bin_rho = 4;
  const double bin_phi = 12;
  BetaGridGenerator* gen = new BetaGridGenerator(min_rho, max_rho, bin_rho,
                                                 bin_phi);
  gen->setDistanceFunction(dist);
  return gen;
}

int main()
{
    std::cout << "Try some serializations..." << std::endl;
    {
      InterestPoint ip;
      OrientedPoint2D op(1.,2.,3.);
      ip.setPosition(op);
      ip.setScale(4.);

      std::string file_name = "output.dat";
      // Save
      {
        std::ofstream ofs(file_name.c_str());
        boost::archive::text_oarchive ar(ofs);
        ar & ip;
      }
      // Load
      InterestPoint ip_restored;
      {
        std::ifstream ifs(file_name.c_str());
        boost::archive::text_iarchive ar (ifs);
        ar & ip_restored;
      }

      std::cout << "ip position : " << ip.getPosition() << std::endl;
      std::cout << "ip_restored position : " << ip_restored.getPosition() << std::endl;
    }
    
    std::cout << "Setting up flirtlib stuff..." << std::endl;
    boost::shared_ptr<SimpleMinMaxPeakFinder> peak_finder_(createPeakFinder());
    boost::shared_ptr<HistogramDistance<double> > histogram_dist_(new SymmetricChi2Distance<double>());
    boost::shared_ptr<Detector> detector_(createDetector(peak_finder_.get()));;
    boost::shared_ptr<DescriptorGenerator> descriptor_(createDescriptor(histogram_dist_.get()));;

    {
      std::cout << "generating a fake laser scan" << std::endl;
      // Generate a vector with readings...
      std::vector<double> phi, rho;
      generateReadings(phi,rho);
      LaserReading reading(phi, rho);
      std::cout << "phi.size(): " << phi.size() << " rho.size() : " << rho.size() << std::endl;
      InterestPointVec pts;
      detector_->detect(reading, pts);
      std::cout << "pts.size() : " << pts.size() << std::endl;
      
      // Check the descriptor content...
      InterestPoint* ip = pts.front();
      ip->setDescriptor(descriptor_->describe(*ip, reading));

      // Save this ip and load it... then use the matching to check the distance.
      const Descriptor* desc_ip = ip->getDescriptor();
      
      std::string file_name = "output2.dat";
      // Save
      {
        std::ofstream ofs(file_name.c_str());
        boost::archive::text_oarchive ar(ofs);
        ar & *ip;
      }
      // Load
      InterestPoint ip_restored;
      {
        std::ifstream ifs(file_name.c_str());
        boost::archive::text_iarchive ar (ifs);
        ar & ip_restored;
      }
      std::cout << "ip position : " << ip->getPosition() << std::endl;
      std::cout << "ip_restored position : " << ip_restored.getPosition() << std::endl;

      // Compute distance between the descriptors?
      std::vector<double> desc, desc_restored;
      std::cout << "ip->getDescriptor(): " << ip->getDescriptor() << std::endl;
      std::cout << "ip_restored.getDescriptor(): " << ip_restored.getDescriptor() << std::endl; 
      ip->getDescriptor()->getFlatDescription(desc);
      ip_restored.getDescriptor()->getFlatDescription(desc_restored);

      std::cout << "desc[0] : " << desc[0] << std::endl;
      std::cout << "desc_restored[0] : " << desc_restored[0] << std::endl;
      
      std::cout << "distance : " << ip->getDescriptor()->distance(ip_restored.getDescriptor()) << std::endl;
      

    }


        {
      std::cout << "generating a fake laser scan" << std::endl;
      // Generate a vector with readings...
      std::vector<double> phi, rho;
      generateReadings(phi,rho);
      LaserReading reading(phi, rho);
      std::cout << "phi.size(): " << phi.size() << " rho.size() : " << rho.size() << std::endl;
      InterestPointVec pts;
      detector_->detect(reading, pts);
      std::cout << "pts.size() : " << pts.size() << std::endl;
      BOOST_FOREACH (InterestPoint* p, pts) 
        p->setDescriptor(descriptor_->describe(*p, reading));

      std::string file_name = "output3.dat";
      // Save
      {
        std::ofstream ofs(file_name.c_str());
        boost::archive::text_oarchive ar(ofs);
        ar & pts;
      }
      // Load
      file_name = "output3.dat2";
      InterestPointVec pts_restored;
      {
        std::ifstream ifs(file_name.c_str());
        boost::archive::text_iarchive ar (ifs);
        ar & pts_restored;
      }
      std::cout << "pts.size() : " << pts.size() << std::endl;
      std::cout << "pts_restored.size() : " << pts_restored.size() << std::endl;

      InterestPoint* ip = pts.front();
      InterestPoint* ip_restored = pts_restored.front();

      {
        
        const BetaGrid *betaGrid = dynamic_cast<const BetaGrid *>(ip->getDescriptor());
        std::cout << "betaGrid : " << betaGrid << std::endl;
        std::cout << "getDistanceFunction() : " << betaGrid->getDistanceFunction() << std::endl;
        
        ip = pts.back();
        const BetaGrid *betaGrid3 = dynamic_cast<const BetaGrid *>(ip->getDescriptor());
        std::cout << "betaGrid3 : " << betaGrid3 << std::endl;
        std::cout << "getDistanceFunction3() : " << betaGrid3->getDistanceFunction() << std::endl;


        BetaGrid *betaGrid2 = dynamic_cast<BetaGrid *>(ip_restored->getDescriptor());
        std::cout << "betaGrid2 : " << betaGrid2 << std::endl;
        std::cout << "getDistanceFunction2() : " << betaGrid2->getDistanceFunction() << std::endl;
        betaGrid2->setDistanceFunction(histogram_dist_.get()); // MUST!!!!!
        

        std::cout << "distance 1 : " << betaGrid->getDistanceFunction()->distance(betaGrid->getHistogram(), betaGrid2->getHistogram()) << std::endl;

        std::cout << "this failes because the get distance function is not loaded from file..." << std::endl;
        std::cout << "distance 2 : " << betaGrid2->getDistanceFunction()->distance(betaGrid->getHistogram(), betaGrid2->getHistogram()) << std::endl;

      }

      std::cout << "distance : " << ip->getDescriptor()->distance(ip_restored->getDescriptor()) << std::endl;

      return 1;
    }


}
     
