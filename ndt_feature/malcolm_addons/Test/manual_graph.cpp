#include "HardCodedG2O.hpp"
#include "ndt_feature/ndtgraph_conversion.h"

int main(){

	ndt_feature::GraphG2OHardCoded hardcoded;

	hardcoded.makeGraph();
	hardcoded.save("harcoded.g2o");

	
}