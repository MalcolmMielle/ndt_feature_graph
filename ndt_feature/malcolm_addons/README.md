# How to use

To create a graph, one can either manual input the graph values (as in HardcodedG2O.hpp) or use files. The files need to respect a format close to the g2o file format. The files for the landmarks and the robot poses are dependant on each other index-wise. First nodes are the robot nodes and after the landmark nodes. Example: robotpose.g2o contains 6 poses (i.e index 0 ->5). Then the first index in landmarks.g2o needs to be 6. Confusingly, the file for the robot prior is independant from the rbot and landmark files index-wise.

To add links between the prior and the robot and landmarks poses, on must use a txt file. The txt file is as follow :

* First a number representing the scale between the prior and the SLAM. It's a scale to go from pixel to m. But in graph, everything is converted to meters and not to pixel. So one has to do `1/scale` before adding a scale to `G2OGraphMaker`  _I know that does not make much sense for now._

* Then the keyword Data [space] nb_of_data. The following data is every prior point in pixel

* Then the keyword Model [space] nb_of_model_point. The following model point from the SLAM are in meters

* Then the keyword Links [space] nb_of_links. The following link data are `data point - model`, all in pixel and meters. This is the only part used by `G2OGraphMaker` the rest correspond to general saving data code I made.

Using those files, one can build a complete graph from the emergency map and the robot map.

See `test_read_g2p.cpp` for an example.