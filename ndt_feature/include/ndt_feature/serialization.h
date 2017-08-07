// #pragma once
// 
// #include <Eigen/Core>
// #include <boost/serialization/split_free.hpp>
// 
// // Forward declaration of class boost::serialization::access
// namespace boost {
//   namespace serialization {
//     class access;
// 
//     template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
//     inline void serialize(
//         Archive & ar, 
//         Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t, 
//         const unsigned int file_version
//     ) 
//     {
//         for(size_t i=0; i<t.size(); i++)
//             ar & t.data()[i];
//     }
// }
