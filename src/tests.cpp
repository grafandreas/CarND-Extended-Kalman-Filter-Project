/*
 * tests.cpp
 *
 *  Created on: 19.01.2018
 *      Author: andreas
 */




#include "kalman_filter.h"
#include "tools.h"
#include "FusionEKF.h"
#include <gtest/gtest.h>
#include <iostream>
#include "Eigen/Dense"

using namespace Eigen;

KalmanFilter * setup() {
	auto sut = new KalmanFilter();
	VectorXd x_(4);
	x_ << 0.0,0.0,1.0,1.0;
	MatrixXd F_ = MatrixXd::Identity(4,4);
	F_(0,2) =0.5;
	F_(1,3) = 0.5;

	MatrixXd P_ = MatrixXd::Zero(4,4);
	MatrixXd Q_ = MatrixXd::Zero(4,4);

	MatrixXd dummy_= MatrixXd::Constant(4,4,0.0) ;
	sut->Init(x_,P_,F_,dummy_,dummy_,Q_);

	return sut;

}

TEST(KalmanTest, Predict) {
   auto sut = setup();
   VectorXd expected(4);
   expected << 0.5,0.5,1.0,1.0;
   sut->Predict();
   ASSERT_EQ(expected,sut->x_)  << sut->x_;



}

TEST(KalmanTest, Update) {
   auto sut = setup();

   sut->Predict();
   sut->Update( Vector4d(1.0,1.0,1.0,1.0));



}


TEST(FusionEKF, calcQZ) {
	auto sut = FusionEKF();
	auto res = sut.calcQ(0.0);
	ASSERT_EQ(MatrixXd::Zero(4,4),res) << res;
}

TEST(FusionEKF, calcQ1) {
	auto sut = FusionEKF();
	auto res = sut.calcQ(1.0);
	for(int i = 0; i < 4; i++) {
		ASSERT_EQ(res(i,0),res(0,i)) << res;
	}
	// Check it
}


TEST(Tools, RMSE1) {
	auto sut = Tools();
	VectorXd t1(4);
	t1 << 1.0,1.0,1.0,2.0;

	std::vector<VectorXd> e;
	std::vector<VectorXd> g;
	e.push_back(t1);
	g.push_back(t1);

	VectorXd exp(4);
	exp << 0.0,0.0,0.0,0.0;
	auto res = sut.CalculateRMSE(e,g);
	ASSERT_EQ(exp,res) << res;
}

TEST(Tools, RMSE2) {
	auto sut = Tools();
	VectorXd t1(4);
	t1 << 1.0,1.0,1.0,2.0;

	VectorXd t2(4);
	t2 << 0.0,1.0,1.0,2.0;

	std::vector<VectorXd> e;
	std::vector<VectorXd> g;
	e.push_back(t1);
	g.push_back(t2);

	VectorXd exp(4);
	exp << 1.0,0.0,0.0,0.0;
	auto res = sut.CalculateRMSE(e,g);
	ASSERT_EQ(exp,res) << res;
}

TEST(Tools, polar2Cart) {
	auto sut = Tools();
	VectorXd t1 = VectorXd::Zero(3);
	t1 << 1.0,0.0,0.0;

	VectorXd exp(4);
	exp << 1.0,0.0,0.0,0.0;
	auto res = sut.polar2Cart(t1);
	ASSERT_EQ(exp,res) << res;
}

TEST(Tools, polar2Cart_2) {
	auto sut = Tools();
	VectorXd t1 = VectorXd::Zero(3);
	t1 << 1.0,M_PI,0.0;

	VectorXd exp(4);
	exp << -1.0,1.2246467991473532e-16,0.0,0.0;
	auto res = sut.polar2Cart(t1);
	ASSERT_DOUBLE_EQ(exp(0),res(0)) << res;
	ASSERT_DOUBLE_EQ(exp(1),res(1)) << res;
}


void PrintTo(const VectorXd& bar, ::std::ostream* os) {
  *os << bar << std::endl;  // whatever needed to print bar to os
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
