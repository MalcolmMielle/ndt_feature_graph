/* *
 * FLIRTLib - Fast Laser Interesting Region Transform Library
 * Copyright (C) 2009-2010 Gian Diego Tipaldi and Kai O. Arras
 *
 * This file is part of FLIRTLib.
 *
 * FLIRTLib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FLIRTLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FLIRTLib.  If not, see <http://www.gnu.org/licenses/>.
 */

#warning Inclusion of a documentation only header...
/**
 * \mainpage FLIRTLib - Fast Laser Interest Region Transform Library
 *
 * \section what What's FLIRTLib?
 * FLIRTLib implements the Fast Laser Interest Region Transform introduced by Tipaldi and Arras. The library implements four different multi-scale 
 * feature detectors and two feature descriptors for 2D range data. It is written in C++ and comes with an API reference (written using Doxygen) and 
 * a set of example binaries to visualize the detector results and the descriptors, as well as perform scan to scan matching. 
 *
 * \subsection detectors Detectors
 * The library implements the following detectors: range, normal edge, normal blob and curvature. All detectors apply the scale-space theory (see \ref references)
 * to the range data. The first three detectors apply the theory to the monodimensional signal defined by the laser scanner, while the last one applies
 * the theory to the continuous geodesic coordinate of the curve that better approximate the point cloud. 
 * \li \e Range \e detector. It finds interest points in scale-space with a blob detector applied on the raw range information in the laser scan.
 * \li \e Normal \e Edge \e detector. It finds interest points in scale-space with an edge detector applied on a local approximation of the normal direction.
 * \li \e Normal \e Blob \e detector. It finds interest points in scale-space with a blob detector applied on a local approximation of the normal direction.
 * \li \e Curvature \e detector. It finds interest points using the scale space theory for curves introduced by Unnikrishnan and Hebert.
 *
 * \subsection descriptors Descriptors
 * \li \e Shape \e Context. It implements a local and linear version of teh Shape Context introduced by Belongie and Malik.
 * \li \e Beta-Grid. It implements a linear-polar occupancy grid. It extends the Shape Context with the notion of free space.
 *
 * \section down Download
 * The source code can be download from OpenSLAM (http://openslam.org/flirtlib.html)
 *
 * The archive has the following structure:
 * \li \c src/ : the source files
 * \li \c data/ : the log files used in the paper.
 * \li \c doc/ : the doxygen configuration file. Run doxygen in this directory and you will obtain a local copy of this html documentaion and the latex reference manual.
 *
 * The \c src/ directory has the following structure:
 * \li \c %mainpage.h : dummy header file for generating the documentation.
 * \li \c INSTALL : installation notes.
 * \li \c COPYING and COPYING.LESSER : the software license. See \ref license for further details.
 * \li \c CMakeLists.txt : the cmake configuration script.
 * \li \c build_tools/ : the directory containing the cmake modules.
 * \li \c feature/ : the directory with the main features code.
 * \li \c gemometry/ : the directory with the code for points.
 * \li \c gui/ : the directory with the code for teh gui and the flirtDemo.
 * \li \c sensors/ : the directory with the code for the sensors abstraction.
 * \li \c sensorstream/ : the directory with the code for the sensor streams abstraction.
 * \li \c utils/ : the directory with the code for utility function such as convolution, regression, pose estimation and histogram distances.
 * \li \c applications/ : the directory with the code for generating the binaries to test the library.
 *
 * \section inst Install
 * Download the library according to \ref down. The library relies on cmake to generate the Makefiles.
 * 
 * Create build directory \verbatim mkdir build && cd build \endverbatim
 * Run cmake and compile \verbatim cmake ../ && make \endverbatim
 * Generate the documentation (optional) \verbatim make doc \endverbatim
 * Install (dafault in /usr/local) \verbatim make install \endverbatim
 *
 * The software depends on the following external libraries
 * \li <em> Boost >= 1.36 (submodules math and graph) </em>
 * \li <em> Qt4 (for the gui)</em> 
 * \li <em> Qwt5 for Qt4 (for the gui)</em> 
 * \li <em> OpenGL (for the gui)</em> 
 * \li <em> Cairo (for drwaing the ransac results)</em> 
 *
 * \section use Usage
 * The library comes with some demo binaries and an API interface.
 * \subsection software Binaries
 * The binaries are compiled out of the source and are available in <install_prefix>/bin. They consists in the following binaries
 * \li \c flirtDemo. It is a simple graphical frontend for visualizing the effect of the different detector and descriptors when changing the parameters.
 *                   To visualize the descriptors simply click on the corresponding interest points in the viewer.
 * \li \c ransacLoopClosureTest. It performs scan to scan matching with ransac. It generates a file with the number of succesful matches for the three different strategies. See the paper for more detail about the ransac experiment.
 * \li \c ransacLoopClosureDraw. It performs scan to scan matching with ransac. It generates a directory with images showing the matching results. It can used to generate animations.
 *
 * \subsection api Using the library
 * The library is compiled as a collection of shared objects and are available in \verbatim <install_prefix>/lib/ \endverbatim. 
 * The header files are located in <install_prefix>/include/flirtlib.  See the class list on the doxygen documentation for a more detailed API reference. 
 *
 * \section references References
 * \li Gian Diego Tipaldi, Manuel Braun, Kai O. Arras. FLIRT: Interest Regions for 2D Range Data with Applications to Robot Navigation. In Proceedings of the International Symposium on Experimental Robotics (ISER). 2010
 * \li Gian Diego Tipaldi, Kai O. Arras. FLIRT -- Interest Regions for 2D Range Data. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA). 2010.
 * \li T. Lindeberg, Scale Space Theory in Computer Vision. Norwell, MA, USA: Kluwer Academic Publishers, 1994.
 * \li R. Unnikrishnan and M. Hebert, “Multi-scale interest regions from unorganized point clouds,” in Workshop on Search in 3D, IEEE Conference on Computer Vision and Pattern Recognition, 2008.
 * \li S. Belongie, J. Malik, and J. Puzicha, “Shape matching and object recognition using shape contexts,” IEEE Transaction on Pattern Analysis and Machine Intelligence, vol. 24, 2002.
 *
 * \section license License
 * 
 * FLIRTLib - Fast Laser Interesting Region Transform Library\n
 * Copyright (C) 2009-2010 Gian Diego Tipaldi and Kai O. Arras\n
 *
 * FLIRTLib is free software: you can redistribute it and/or modify\n
 * it under the terms of the GNU Lesser General Public License as published by\n
 * the Free Software Foundation, either version 3 of the License, or\n
 * (at your option) any later version.\n
 *
 * FLIRTLib is distributed in the hope that it will be useful,\n
 * but WITHOUT ANY WARRANTY; without even the implied warranty of\n
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n
 * GNU Lesser General Public License for more details.\n
 *
 * You should have received a copy of the GNU Lesser General Public License\n
 * along with FLIRTLib.  If not, see \htmlonly <a rel="license" href="http://www.gnu.org/licenses/"> \endhtmlonly http://www.gnu.org/licenses/ \htmlonly </a> \endhtmlonly .
 *
 * 
 */

