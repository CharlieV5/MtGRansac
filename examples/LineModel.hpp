#pragma once

#include "RANSAC.hpp"

class Point2D 
{
public:
	Point2D(double x_, double y_)
	{
		x = x_;
		y = y_;
	};

	double x;
	double y;
};

class Line2DModel : public ransac::Model<Point2D>
{
protected:
	// Parametric form
	double A, B, C; // Ax + By + C = 0
	double m_dist_denominator; // = sqrt(A^2 + B^2). Stored for efficiency reasons

public:

	void getParams(std::vector<double>& params)
	{
		params.clear();
		params.resize(3);
		params[0] = A;
		params[1] = B;
		params[2] = C;
	}

	virtual void initModel(const std::vector<std::shared_ptr<Point2D>> &v_data) override
	{
		if (v_data.size() != 2)
			throw std::runtime_error("Line2DModel - Number of input parameters does not match minimum number required for this model.");
				
		auto p1 = v_data[0];
		auto p2 = v_data[1];

		// Compute the line parameters
		A = p2->y - p1->y;
		B = p1->x - p2->x;
		C = p2->x*p1->y - p1->x*p2->y;

		m_dist_denominator = sqrt(A * A + B * B); // Cache square root for efficiency
	};

	virtual double computeDistanceMeasure(std::shared_ptr<Point2D> p) override
	{
		// Return distance between passed "point" and this line
		return fabs(A * p->x + B * p->y + C) / m_dist_denominator;	
	};

};

