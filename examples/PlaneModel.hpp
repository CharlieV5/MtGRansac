#pragma once

#include "RANSAC.hpp"

class Point3D 
{
public:
	Point3D(double x_, double y_, double z_)
	{
		x = x_;
		y = y_;
		z = z_;
	};

	double x;
	double y;
	double z;
};

class PlaneModel : public ransac::Model<Point3D>
{

public:
	
	void getParams(std::vector<double>& params)
	{
		params.clear();
		params.resize(4);
		params[0] = A;
		params[1] = B;
		params[2] = C;
		params[3] = D;
	}

	virtual void initModel(const std::vector<std::shared_ptr<Point3D>> &v_data) override
	{
		if (v_data.size() != 3)
			throw std::runtime_error("PlaneModel - Number of input points does not match minimum number required for this model.");

		// Check for AbstractParamter types
		auto p1 = v_data[0];
		auto p2 = v_data[1];
		auto p3 = v_data[2];

		// Compute the plane parameters
		// the vector, start from pt1 to pt2
		double ax = p2->x - p1->x;
		double ay = p2->y - p1->y;
		double az = p2->z - p1->z;

		// the vector, start from pt1 to pt3
		double bx = p3->x - p1->x;
		double by = p3->y - p1->y;
		double bz = p3->z - p1->z;

		// cross product to get the normal of plane
		A = ay*bz - az*by;
		B = az*bx - ax*bz;
		C = ax*by - ay*bx;

		// Ax + By + Cx + D = 0
		D = -(A*p1->x + B*p1->y + C*p1->z);

		m_dist_denominator = sqrt(A * A + B * B + C * C); // Cache square root for efficiency
	};
	
	virtual double computeDistanceMeasure(std::shared_ptr<Point3D> p) override
	{
		// Return distance of "point" to plane
		return fabs(A * p->x + B * p->y + C * p->z + D) / m_dist_denominator;
	};

protected:
	// Parametric form
	double A, B, C, D; // Ax + By + Cx + D = 0
	double m_dist_denominator; // = sqrt(A^2 + B^2 + C^2). Stored for efficiency reasons

};

