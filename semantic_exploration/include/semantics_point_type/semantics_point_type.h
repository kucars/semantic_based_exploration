/**
BSD 3-Clause License

Copyright (c) 2018, Khalifa University Robotics Institute
Copyright (c) 2018, Tarek Taha tarek@tarektaha.com
Copyright (c) 2018, Reem Ashour reemashour1@gmail.com
Copyright (c) 2020, Mohamed Abdelkader mohamedashraf123@gmail.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef SEMANTICS_POINT_TYPE
#define SEMANTICS_POINT_TYPE
// Reference http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
/**
 * \brief Point type contains XYZ RGB, 3 most confident semantic colors and their confidences
 * \author Xuan Zhang
 * \data Mai-July 2018
 */

struct PointXYZRGBSemanticsMax
{
    PCL_ADD_POINT4D;  // Preferred way of adding a XYZ+padding
    PCL_ADD_RGB;
    union  // Semantic color
    {
        float semantic_color;
    };
    union  // Confidences
    {
        float confidence;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                     // enforce SSE padding for correct memory alignment

// here we assume a XYZ + RGB + "sementic_color" + "confidence" (as fields)
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBSemanticsMax,
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)
				  (float, semantic_color, semantic_color)(float, confidence,confidence))

struct PointXYZRGBSemanticsBayesian
{
    PCL_ADD_POINT4D;  // Preferred way of adding a XYZ+padding
    PCL_ADD_RGB;
    union  // Semantic colors
    {
        float data_sem[4];
        struct
        {
            float semantic_color1;
            float semantic_color2;
            float semantic_color3;
        };
    };
    union  // Confidences
    {
        float data_conf[4];
        struct
        {
            float confidence1;
            float confidence2;
            float confidence3;
        };
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                     // enforce SSE padding for correct memory alignment

// here we assume a XYZ + RGB + "sementic_colors" + "confidences" (as fields)
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRGBSemanticsBayesian,
    (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, semantic_color1,
                                                             semantic_color1)(
        float, semantic_color2, semantic_color2)(float, semantic_color3, semantic_color3)(
        float, confidence1, confidence1)(float, confidence2, confidence2)(float, confidence3,
                                                                          confidence3))
#endif
