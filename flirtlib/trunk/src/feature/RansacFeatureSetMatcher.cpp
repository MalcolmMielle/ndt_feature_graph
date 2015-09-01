//
//
// FLIRTLib - Fast Laser Interesting Region Transform Library
// Copyright (C) 2009-2010 Gian Diego Tipaldi and Kai O. Arras
//
// This file is part of FLIRTLib.
//
// FLIRTLib is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// FLIRTLib is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with FLIRTLib.  If not, see <http://www.gnu.org/licenses/>.
//

#include "RansacFeatureSetMatcher.h"

#include <boost/random.hpp>
#include <boost/random/uniform_smallint.hpp>
#include <sys/time.h>


RansacFeatureSetMatcher::RansacFeatureSetMatcher(double acceptanceThreshold, double successProbability, double inlierProbability, double distanceThreshold, double rigidityThreshold, bool adaptive):
    AbstractFeatureSetMatcher(acceptanceThreshold),
    m_successProbability(successProbability),
    m_inlierProbability(inlierProbability),
    m_distanceThreshold(distanceThreshold),
    m_rigidityThreshold(rigidityThreshold),
    m_adaptive(adaptive)
{

}

double RansacFeatureSetMatcher::matchSets(const std::vector<InterestPoint *> &reference, const std::vector<InterestPoint *> &data, OrientedPoint2D& transformation) const
{
    std::vector< std::pair<InterestPoint *, InterestPoint *> > correspondences;
    return matchSets(reference, data, transformation, correspondences);
}

double RansacFeatureSetMatcher::matchSets(const std::vector<InterestPoint *> &reference, const std::vector<InterestPoint *> &data, OrientedPoint2D& transformation,
					  std::vector< std::pair<InterestPoint *, InterestPoint *> > &correspondences) const
{
    correspondences.clear();
    unsigned int iterations = m_adaptive ? 1e17 : ceil(log(1. - m_successProbability)/log(1. - m_inlierProbability * m_inlierProbability));
    
    // Compute possible correspondences based on 1-NN thresholding
   std::vector< std::pair<InterestPoint *, InterestPoint *> > possibleCorrespondences;
   for(unsigned int i = 0; i < data.size(); i++){
	double minCorrespondenceDistance = 1e17;
	unsigned int minCorrespondenceIndex = 0;
	for(unsigned int j = 0; j < reference.size(); j++){
	    double distance = data[i]->getDescriptor()->distance(reference[j]->getDescriptor());
	    if(distance < minCorrespondenceDistance){
		minCorrespondenceDistance = distance;
		minCorrespondenceIndex = j;
	    }
	}
	if(minCorrespondenceDistance < m_distanceThreshold){
	    possibleCorrespondences.push_back(std::make_pair(data[i], reference[minCorrespondenceIndex]));
	}
    }
    
    // Check if there are enough absolute matches 
    if(possibleCorrespondences.size() < 2){  
// 	std::cout << "Not enough possible correspondences" << std::endl;
	return 1e17;
    }
    
    // Check if there are enough matches compared to the inlier probability 
    if(double(possibleCorrespondences.size()) * m_inlierProbability < 2){  
// 	std::cout << "Not enough possible correspondences for the inlier probability" << std::endl;
	return 1e17;
    }
    
    boost::mt19937 rng;
    boost::uniform_smallint<int> generator(0, possibleCorrespondences.size() - 1);
    
    // Main loop
    double minimumScore = 1e17;
    for(unsigned int i = 0; i < iterations; i++){
// 	std::cout << "\tIteration " << i << std::endl;
	unsigned int first = generator(rng);
	unsigned int second = generator(rng);
	while(second == first) second = generator(rng); // avoid useless samples
	std::pair< std::pair<InterestPoint *, InterestPoint *>, std::pair<InterestPoint *, InterestPoint *> > minimumSampleSet(possibleCorrespondences[first], possibleCorrespondences[second]);
	
	// Test rigidity
	const Point2D& diffFirst = possibleCorrespondences[first].first->getPosition() - possibleCorrespondences[second].first->getPosition();
	const Point2D& diffSecond = possibleCorrespondences[first].second->getPosition() - possibleCorrespondences[second].second->getPosition();
	double distanceFirst = diffFirst * diffFirst;
	double distanceSecond = diffSecond * diffSecond;
	if((distanceFirst - distanceSecond)*(distanceFirst - distanceSecond)/(8*(distanceFirst + distanceSecond	)) > m_rigidityThreshold){
// 	    std::cout << "\t\tRigidity failure" << std::endl;
	    continue;
	}
	
	// Compute hypothesis
	std::vector< std::pair<InterestPoint *, InterestPoint *> > inlierSet;
	OrientedPoint2D hypothesis = generateHypothesis(minimumSampleSet);
	
	// Verify hypothesis
	double score = verifyHypothesis(reference, data, hypothesis, inlierSet);
	if(score < minimumScore){
	    minimumScore = score;
	    transformation = hypothesis;
	    correspondences = inlierSet;
	    
	    // Adapt the number of iterations
	    if (m_adaptive){
		double inlierProbability = double(correspondences.size())/double(data.size());
		iterations = ceil(log(1. - m_successProbability)/log(1. - inlierProbability * inlierProbability));
	    }
	}
    }
    std::vector<std::pair<Point2D, Point2D> > pointCorrespondences(correspondences.size());
    for(unsigned int i = 0; i < correspondences.size(); i++){
	pointCorrespondences[i] = std::make_pair(correspondences[i].first->getPosition(), correspondences[i].second->getPosition());
    }
    compute2DPose(pointCorrespondences, transformation);
    return verifyHypothesis(reference, data, transformation, correspondences);
}
