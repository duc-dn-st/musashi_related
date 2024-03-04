// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#ifndef MPCC_CONFIG_H
#define MPCC_CONFIG_H

#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace mpcc{

    // #define MAX(a,b) (a < b) ? b : a

    #define NX 6
    #define NU 3

    #define NB 9 // max number of bounds
    #define NPC 1 // number of polytopic constraints (= NS)
    #define NS 1 // number of soft constraints

    static constexpr int N = 60;   //change 60sw
    static constexpr double INF = 1E5;
    static constexpr int N_SPLINE = 5000;


    struct StateInputIndex{
        int X = 0;
        int Y = 1;
        int phi = 2;
        int s = 3;
        int vx = 4;     // lin vel
        int vs = 5;     // arc lenth vel

        int dVx = 0; 
        int dPhi = 1;
        int dVs = 2;

        int con_track = 0;
    };

    static const StateInputIndex si_index;

}
#endif //MPCC_CONFIG_H
