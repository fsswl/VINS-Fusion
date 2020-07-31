/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>

template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
	q_inverse[0] = q[0];
	q_inverse[1] = -q[1];
	q_inverse[2] = -q[2];
	q_inverse[3] = -q[3];
};


struct TError
{
	TError(double t_x, double t_y, double t_z, double var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), var(var){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var);
		residuals[1] = (tj[1] - T(t_y)) / T(var);
		residuals[2] = (tj[2] - T(t_z)) / T(var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          TError, 3, 3>(
	          	new TError(t_x, t_y, t_z, var)));
	}

	double t_x, t_y, t_z, var;

};

struct RelativeRTError
{
	RelativeRTError(double t_x, double t_y, double t_z, 
					double q_w, double q_x, double q_y, double q_z,
					double t_var, double q_var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), 
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   t_var(t_var), q_var(q_var){}

	template <typename T>
	bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		T i_q_w[4];
		QuaternionInverse(w_q_i, i_q_w);

		T t_i_ij[3];
		ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
		residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
		residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

		T relative_q[4];
		relative_q[0] = T(q_w);
		relative_q[1] = T(q_x);
		relative_q[2] = T(q_y);
		relative_q[3] = T(q_z);

		T q_i_j[4];
		ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

		T relative_q_inv[4];
		QuaternionInverse(relative_q, relative_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q); 

		residuals[3] = T(2) * error_q[1] / T(q_var);
		residuals[4] = T(2) * error_q[2] / T(q_var);
		residuals[5] = T(2) * error_q[3] / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var, const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          RelativeRTError, 6, 4, 3, 4, 3>(
	          	new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}

	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var;

};

struct exRelativeRTError
{
	exRelativeRTError(double t_x_vio, double t_y_vio, double t_z_vio,
		              double q_w_vio, double q_x_vio, double q_y_vio, double q_z_vio,
		            double t_x_gps, double t_y_gps, double t_z_gps,
					double t_var)
				  :t_x_vio(t_x_vio), t_y_vio(t_y_vio), t_z_vio(t_z_vio),
				   q_w_vio(q_w_vio), q_x_vio(q_x_vio), q_y_vio(q_y_vio), q_z_vio(q_z_vio),
				   t_x_gps(t_x_gps), t_y_gps(t_y_gps), t_z_gps(t_z_gps),
				   t_var(t_var){}

	template <typename T>
	bool operator()(const T* q, const T* t, const T* t_ex_vio_gps, T* residuals) const
	{
		T t_vio[3];
		t_vio[0] = T(t_x_vio);
		t_vio[1] = T(t_y_vio);
		t_vio[2] = T(t_z_vio);

		T q_vio[4];
		q_vio[0] = T(q_w_vio);
		q_vio[1] = T(q_x_vio);
		q_vio[2] = T(q_y_vio);
		q_vio[3] = T(q_z_vio);

		T t_world_gps[3];
		ceres::QuaternionRotatePoint(q_vio, t_ex_vio_gps, t_world_gps);
		t_world_gps[0] = t_world_gps[0] + t_vio[0];
		t_world_gps[1] = t_world_gps[1] + t_vio[1];
		t_world_gps[2] = t_world_gps[2] + t_vio[2];

		T t_vio2gps[3];
		ceres::QuaternionRotatePoint(q, t_world_gps, t_vio2gps);

		residuals[0] = (T(t_x_gps) - t_vio2gps[0] - t[0]) / T(t_var);
		residuals[1] = (T(t_y_gps) - t_vio2gps[1] - t[1]) / T(t_var);
		residuals[2] = (T(t_z_gps) - t_vio2gps[2] - T(0)) / T(t_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x_vio, const double t_y_vio, const double t_z_vio,
		                               const double q_w_vio, const double q_x_vio, const double q_y_vio, const double q_z_vio,
									   const double t_x_gps, const double t_y_gps, const double t_z_gps,
									   const double t_var)
	{
	  return (new ceres::AutoDiffCostFunction<
	          exRelativeRTError, 3, 4, 2, 3>(new exRelativeRTError(t_x_vio, t_y_vio, t_z_vio,
	                                q_w_vio, q_x_vio, q_y_vio, q_z_vio,
	                                t_x_gps, t_y_gps, t_z_gps, t_var)));
	}

	double t_x_vio, t_y_vio, t_z_vio;
	double q_w_vio, q_x_vio, q_y_vio, q_z_vio;
	double t_x_gps, t_y_gps, t_z_gps;
	double t_var;

};