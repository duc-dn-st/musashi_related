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

#include "constraints.h"
namespace mpcc{
Constraints::Constraints()
{   
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Constraints::Constraints(double Ts,const PathToJson &path) 
:model_(Ts,path),
param_(Param(path.param_path))
{
}

OneDConstraint Constraints::getTrackConstraints(const ArcLengthSpline &track,const State &x) const
{
    // given arc length s and the track -> compute linearized track constraints
    const double s = x.s;

    // X-Y point of the center line
    const Eigen::Vector2d pos_center = track.getPostion(s);
    const Eigen::Vector2d d_center   = track.getDerivative(s);
    // Tangent of center line at s
    const Eigen::Vector2d tan_center = {-d_center(1),d_center(0)};
    double theta = atan2(d_center(1),d_center(0));

    // double length = (std::sqrt((param_.car_w * param_.car_w) + (param_.car_l * param_.car_l)));

    // TODO use info of real path boundaries(This is Only For Testing)
    // -compare 2 point to check if for corner and which direction, then apply constraints
    double out;
    double in;

    if(std::abs(theta) < 0.001 && s < 4.0)
    {
        out = param_.r_out - param_.car_w;
        in = param_.r_in - param_.car_w;
    }
    else if(std::abs(theta) >= 1.57)
    {
        out = 0.25 - param_.car_w;
        in = 0.25 - param_.car_w;
    }
    else
    {
        out = 0.25 - param_.car_l;
        in = 0.25 - param_.car_w;
    }

    // inner and outer track boundary given left and right width of track
    // TODO make R_out and R_in dependent on s
    const Eigen::Vector2d pos_outer = pos_center + out*tan_center;
    const Eigen::Vector2d pos_inner = pos_center - in*tan_center;

    std::cout << "theta" << theta << std::endl;

    // Define track Jacobian as Perpendicular vector
    C_i_MPC C_track_constraint = C_i_MPC::Zero();
    C_track_constraint(0,0) = tan_center(0);
    C_track_constraint(0,1) = tan_center(1);
    // Compute bounds
    const double track_constraint_lower = tan_center(0)*pos_inner(0) + tan_center(1)*pos_inner(1);
    const double track_constraint_upper = tan_center(0)*pos_outer(0) + tan_center(1)*pos_outer(1);

    return {C_track_constraint,track_constraint_lower,track_constraint_upper};
}

ConstrainsMatrix Constraints::getConstraints(const ArcLengthSpline &track,const State &x,const Input &u) const
{
    // compute all the polytopic state constraints
    // compute the three constraints

    ConstrainsMatrix constrains_matrix;
    const OneDConstraint track_constraints = getTrackConstraints(track,x);
    //const OneDConstraint tire_constraints_rear = getTireConstraintRear(x);
    //const OneDConstraint alpha_constraints_front = getAlphaConstraintFront(x);

    C_MPC C_constrains_matrix;
    d_MPC dl_constrains_matrix;
    d_MPC du_constrains_matrix;

    C_constrains_matrix.row(si_index.con_track) = track_constraints.C_i;
    dl_constrains_matrix(si_index.con_track) = track_constraints.dl_i;
    du_constrains_matrix(si_index.con_track) = track_constraints.du_i;

    //C_constrains_matrix.row(si_index.con_tire) = tire_constraints_rear.C_i;
    //dl_constrains_matrix(si_index.con_tire) = tire_constraints_rear.dl_i;
    //du_constrains_matrix(si_index.con_tire) = tire_constraints_rear.du_i;

    //C_constrains_matrix.row(si_index.con_alpha) = alpha_constraints_front.C_i;
    //dl_constrains_matrix(si_index.con_alpha) = alpha_constraints_front.dl_i;
    //du_constrains_matrix(si_index.con_alpha) = alpha_constraints_front.du_i;

    return {C_constrains_matrix,D_MPC::Zero(),dl_constrains_matrix,du_constrains_matrix};
}
}