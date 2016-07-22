#include "HardCodedG2O.hpp"


int main(){

	ndt_feature::GraphG2OHardCoded hardcoded;

	hardcoded.makeGraph();
	hardcoded.save("harcoded.g2o");

	
}