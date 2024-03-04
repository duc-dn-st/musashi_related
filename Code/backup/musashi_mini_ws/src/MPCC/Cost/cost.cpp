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

#include "cost.h"
namespace mpcc{
Cost::Cost() 
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Cost::Cost(const PathToJson &path) 
:cost_param_(CostParam(path.cost_path)),
param_(Param(path.param_path))
{
}

TrackPoint Cost::getRefPoint(const ArcLengthSpline &track,const State &x) const
{
    // compute all the geometry information of the track at a given arc length
    const double s = x.s;

    // X-Y postion of the reference at s
    const Eigen::Vector2d pos_ref = track.getPostion(s);
    const double x_ref = pos_ref(0);
    const double y_ref = pos_ref(1);
    // reference path derivatives
    const Eigen::Vector2d dpos_ref = track.getDerivative(s);
    const double dx_ref = dpos_ref(0);
    const double dy_ref = dpos_ref(1);
    // angle of the reference path
    const double theta_ref = atan2(dy_ref,dx_ref);
    // second order derivatives
    Eigen::Vector2d ddpos_ref = track.getSecondDerivative(s);
    const double ddx_ref = ddpos_ref(0);
    const double ddy_ref = ddpos_ref(1);
    // curvature
    double dtheta_ref_nom = (dx_ref*ddy_ref - dy_ref*ddx_ref);
    double dtheta_ref_denom = (dx_ref*dx_ref + dy_ref*dy_ref);
    if(std::fabs(dtheta_ref_nom) < 1e-7)
        dtheta_ref_nom = 0;
    if(std::fabs(dtheta_ref_denom) < 1e-7)
        dtheta_ref_denom = 1e-7;
    double dtheta_ref = dtheta_ref_nom/dtheta_ref_denom;

    return {x_ref,y_ref,dx_ref,dy_ref,theta_ref,dtheta_ref};
}

ErrorInfo Cost::getErrorInfo(const ArcLengthSpline &track,const State &x) const
{
    ErrorInfo error_info;
    // compute error between reference and X-Y position of the car
    const double X = x.X;
    const double Y = x.Y;
    const TrackPoint track_point = getRefPoint(track,x);
    // contouring  error
    Eigen::Matrix<double,1,2> contouring_error;
    contouring_error(0) =  std::sin(track_point.theta_ref)*(X - track_point.x_ref) -
                           std::cos(track_point.theta_ref)*(Y - track_point.y_ref); 
    // lag error
    contouring_error(1) =  - std::cos(track_point.theta_ref)*(X - track_point.x_ref) -
                           std::sin(track_point.theta_ref)*(Y - track_point.y_ref); 
                           
    // partial derivatives of the lag and contouring error with respect to s(theta_ref)
    Eigen::Matrix<double,2,NX> d_contouring_error = Eigen::Matrix<double,2,NX>::Zero();

    const double dContouringError = - track_point.dtheta_ref*std::cos(track_point.theta_ref)*(track_point.x_ref - X)
                                    - track_point.dtheta_ref*std::sin(track_point.theta_ref)*(track_point.y_ref - Y)
                                    - track_point.dx_ref*std::sin(track_point.theta_ref)
                                    + track_point.dy_ref*std::cos(track_point.theta_ref);
    const double dLagError =        - track_point.dtheta_ref*std::sin(track_point.theta_ref)*(track_point.x_ref - X)
                                    + track_point.dtheta_ref*std::cos(track_point.theta_ref)*(track_point.y_ref - Y)
                                    + track_point.dx_ref*std::cos(track_point.theta_ref)
                                    + track_point.dy_ref*std::sin(track_point.theta_ref);

    // compute all remaining partial derivatives and store the in dError
    d_contouring_error(0,si_index.X) =  std::sin(track_point.theta_ref); // dControuring_err / dX
    d_contouring_error(0,si_index.Y) = -std::cos(track_point.theta_ref); // dControuring_err / dY
    d_contouring_error(0,si_index.s) = dContouringError;  // dControuring_err / ds

    d_contouring_error(1,si_index.X) = -std::cos(track_point.theta_ref);    // dLag_err / dX
    d_contouring_error(1,si_index.Y) = -std::sin(track_point.theta_ref);    // dLag_err / dY
    d_contouring_error(1,si_index.s) = dLagError;   // dLag_err / ds

    return {contouring_error,d_contouring_error};
}

CostMatrix Cost::getContouringCost(const ArcLengthSpline &track, const State &x,const int k) const
{
    // compute state cost, formed by contouring error cost + cost on "real" inputs
    // compute reference information
    const StateVector x_vec = stateToVector(x);
    // compute error and jacobean of error
    const ErrorInfo error_info = getErrorInfo(track,x);
    // contouring cost matrix
    Eigen::Vector2d ContouringCost;
    ContouringCost.setZero(2);
    ContouringCost(0) = k < N ? cost_param_.q_c : cost_param_.q_c_N_mult * cost_param_.q_c;
    ContouringCost(1) = cost_param_.q_l;
    // contouring and lag error part
    Q_MPC Q_contouring_cost = Q_MPC::Zero();
    q_MPC q_contouring_cost = q_MPC::Zero();

    Eigen::Matrix<double,1,NX> d_contouring_error = Eigen::Matrix<double,1,NX>::Zero();
    d_contouring_error = error_info.d_error.row(0);
    const double contouring_error_zero = error_info.error(0) - d_contouring_error*stateToVector(x);

    Eigen::Matrix<double,1,NX> d_lag_error = Eigen::Matrix<double,1,NX>::Zero();
    d_lag_error = error_info.d_error.row(1);
    const double lag_error_zero = error_info.error(1) - d_lag_error*stateToVector(x);

    Q_contouring_cost = ContouringCost(0)*d_contouring_error.transpose()*d_contouring_error +
                        ContouringCost(1)*d_lag_error.transpose()*d_lag_error;
    // regularization cost on yaw rate
    //Q_contouring_cost(si_index.r, si_index.r) = k < N ? cost_param_.q_r : cost_param_.q_r_N_mult * cost_param_.q_r;
    Q_contouring_cost = 2.0*Q_contouring_cost;


    q_contouring_cost = ContouringCost(0)*2.0*contouring_error_zero*d_contouring_error.transpose() +
                        ContouringCost(1)*2.0*lag_error_zero*d_lag_error.transpose();
    
    // progress maximization part
    q_contouring_cost(si_index.vs) = -cost_param_.q_vs;

    // DEBUG
    /*if(k == N-1){
        std::cout << "ContouringCost: " << std::endl;
        std::cout << std::setprecision(6); 
        std::cout << "[";
        for(int i = 0; i < Q_contouring_cost.cols(); i++)
            std::cout << " [ " << Q_contouring_cost.row(i) << " ] " << std::endl;
        std::cout << "]" << std::endl;
        
        const TrackPoint track_point = getRefPoint(track,x);
        const double X = x.X;
        const double Y = x.Y;
        std::cout << "[Track] x_ref - X: " << track_point.x_ref - X<< ", y_ref - Y: " << track_point.y_ref - Y << ", theta_ref: " << track_point.theta_ref  << std::endl;
        std::cout << "contouring_error: " << error_info.error(0) << ", lag_error: " << error_info.error(1) << std::endl;
        std::cout << "contouring_error_zero: " << contouring_error_zero << ", lag_error_zero: " << lag_error_zero << std::endl;
    }*/
    // solver interface expects 0.5 x^T Q x + q^T x
    return {Q_contouring_cost,R_MPC::Zero(),S_MPC::Zero(),q_contouring_cost,r_MPC::Zero(),Z_MPC::Zero(),z_MPC::Zero()};
}

CostMatrix Cost::getHeadingCost(const ArcLengthSpline &track, const State &x) const
{
    // get heading of the track
    const Eigen::Vector2d dpos_ref = track.getDerivative(x.s);
    const double dx_ref = dpos_ref(0);
    const double dy_ref = dpos_ref(1);
    // angle of the reference path
    double theta_ref = atan2(dy_ref, dx_ref);
    //theta_ref += 2.0*M_PI*std::round((x.phi - theta_ref)/(2.0*M_PI));
    double theta_error = theta_ref - x.phi;
    //std::cout << "theta_ref: " << theta_ref << std::endl;

    if (theta_error < -M_PI) 
        theta_error = M_PI - abs(theta_error + M_PI);
    

    // if(std::fabs(x.phi - theta_ref)>= 1.5){
    //     std::cout << "k" << std::endl;
    // }


    Q_MPC Q_heading_cost = Q_MPC::Zero();
    Q_heading_cost(si_index.phi,si_index.phi) = 2.0*cost_param_.q_theta;
    q_MPC q_heading_cost = q_MPC::Zero();
    q_heading_cost(si_index.phi) = - cost_param_.q_theta*theta_error;

    // std::cout << "theta_error = " << theta_error << std::endl;
    //std::cout << "cost_param_.q_theta: " << cost_param_.q_theta << std::endl;

    return {Q_heading_cost,R_MPC::Zero(),S_MPC::Zero(),q_heading_cost,r_MPC::Zero(),Z_MPC::Zero(),z_MPC::Zero()};
}

CostMatrix Cost::getInputCost() const
{
    // input cost and rate of chagen of real inputs
    //Q_MPC Q_input_cost = Q_MPC::Zero();
    R_MPC R_input_cost = R_MPC::Zero();
    // cost of "real" inputs
    //Q_input_cost(si_index.vx,si_index.vx) = cost_param_.r_D;
    //Q_input_cost(si_index.phi,si_index.phi) = cost_param_.r_delta;
    //Q_input_cost(si_index.vs,si_index.vs) = cost_param_.r_vs;
    // quadratic part
    R_input_cost(si_index.dVx,si_index.dVx) = cost_param_.r_dVx;
    R_input_cost(si_index.dPhi,si_index.dPhi) = cost_param_.r_dPhi;
    R_input_cost(si_index.dVs,si_index.dVs) = cost_param_.r_dVs;
    // solver interface expects 0.5 u^T R u + r^T u
    //Q_input_cost = 2.0*Q_input_cost;
    R_input_cost = 2.0*R_input_cost;

    return {Q_MPC::Zero(), R_input_cost,S_MPC::Zero(),q_MPC::Zero(),r_MPC::Zero(),Z_MPC::Zero(),z_MPC::Zero()};
}

CostMatrix Cost::getSoftConstraintCost() const
{
    // input cost and rate of chagen of real inputs
    Z_MPC Z_cost = Z_MPC::Identity();
    z_MPC z_cost = z_MPC::Ones();
    // cost of "real" inputs

    Z_cost(si_index.con_track,si_index.con_track) = cost_param_.sc_quad_track;

    z_cost(si_index.con_track) = cost_param_.sc_lin_track;

    return {Q_MPC::Zero(),R_MPC::Zero(),S_MPC::Zero(),q_MPC::Zero(),r_MPC::Zero(),Z_cost,z_cost};
}

CostMatrix Cost::getCost(const ArcLengthSpline &track, const State &x,const int k) const
{
    // generate quadratic cost function
    const CostMatrix contouring_cost = getContouringCost(track,x,k);
    const CostMatrix heading_cost = getHeadingCost(track,x);
    const CostMatrix input_cost = getInputCost();
    const CostMatrix soft_con_cost = getSoftConstraintCost();

    Q_MPC Q_not_sym = contouring_cost.Q + input_cost.Q + heading_cost.Q; // + beta_cost.Q;
    Q_MPC Q_reg = 1e-9*Q_MPC::Identity();

    /* 
    std::cout << "Num(Q_MPC.cols): " << Q_not_sym.cols() << std::endl;
    std::cout << "Num(Q_MPC.rows): " << Q_not_sym.rows() << std::endl;
    
    std::cout << "[";
    for(int i = 0; i < Q_not_sym.cols(); i++)
        std::cout << " [ " << Q_not_sym.row(i) << " ] " << std::endl;
    std::cout << "]" << std::endl; 
    */
    
    const Q_MPC Q = 0.5*(Q_not_sym.transpose()+Q_not_sym);// + Q_reg;//contouring_cost.Q + input_cost.Q + beta_cost.Q;
    const R_MPC R = contouring_cost.R + input_cost.R + heading_cost.R; // + beta_cost.R;
    const q_MPC q = contouring_cost.q + input_cost.q + heading_cost.q; // + beta_cost.q;
    const r_MPC r = contouring_cost.r + input_cost.r + heading_cost.r; // + beta_cost.r;
    const Z_MPC Z = soft_con_cost.Z;
    const z_MPC z = soft_con_cost.z;


    return {Q,R,S_MPC::Zero(),q,r,Z,z};
}
}