#include <iostream>
#include <cmath>
#include <random>
#include <cstdint>
#include <time.h>

#include "PlaneModel.hpp"
#include "LineModel.hpp"
using namespace std;

void createRandomPlanePoints(vector<shared_ptr<Point3D>>& points)
{
	int thickness = 5;
	int Size = 100;

	// Randomly generate points in a 3D plane  for testing
	random_device SeedDevice;
	mt19937 RNG = mt19937(SeedDevice());

	uniform_int_distribution<int> UniDist(0, thickness - 1); // [Incl, Incl]
	int Perturb = 2;
	normal_distribution<double> PerturbDist(0, Perturb);

	FILE* fp = fopen("plane_random_points.txt", "w");
	
	double a = sqrt(2)*0.5, b = 0, c = sqrt(2)*0.5, d = -1;
	for (int j = 0; j < Size; ++j)
	{
		for (int i = 0; i < Size; ++i)
		{
			double x = i;
			double y = j;
			double z = -(a*x + b*y + d) / c + PerturbDist(RNG);
			shared_ptr<Point3D> CandPt = make_shared<Point3D>(x, y, z);
			points.push_back(CandPt);

			fprintf(fp, "%lf,%lf,%lf\n", x, y, z);
		}
	}

	fclose(fp);
}

void planeFittingExample()
{	
	vector<shared_ptr<Point3D>> points;
	createRandomPlanePoints(points);

	ransac::RANSAC<PlaneModel, Point3D, 3> ransac_estimator;
	ransac_estimator.initialize(0.5, 1000, 0.99); // Threshold, iterations
	clock_t start = clock();
	ransac_estimator.estimate(points);
	int64_t end = clock();
	cout << "RANSAC took: " << (double)(end - start) / CLOCKS_PER_SEC * 1000.0 << " ms." << endl;

	FILE* fp2 = fopen("plane_inlier_points.txt", "w");
	
	for (auto& i : ransac_estimator.getInlierIndices())
		fprintf(fp2, "%lf,%lf,%lf\n", points[i]->x, points[i]->y, points[i]->z);

	fclose(fp2);

	FILE* fp3 = fopen("plane.txt", "w");
	auto model = ransac_estimator.getBestModel();
	if (model)
	{
		std::vector<double> params;
		model->getParams(params);
		for (auto p : params)
			fprintf(fp3, "%lf\n", p);
	}
	fclose(fp3);	
}

void createRandomLinePoints(vector<shared_ptr<Point2D>>& points)
{
	int Side = 1000;
	int nPoints = 500;

	// Randomly generate points in a 2D plane roughly aligned in a line for testing
	random_device SeedDevice;
	mt19937 RNG = mt19937(SeedDevice());

	uniform_int_distribution<int> UniDist(0, Side - 1); // [Incl, Incl]
	int Perturb = 25;
	normal_distribution<double> PerturbDist(0, Perturb);

	FILE* fp = fopen("line_random_points.txt", "w");
	for (int i = 0; i < nPoints; ++i)
	{
		int Diag = UniDist(RNG);
		double x = floor(Diag + PerturbDist(RNG));
		double y = floor(Diag + PerturbDist(RNG));

		shared_ptr<Point2D> CandPt = make_shared<Point2D>(x, y);
		points.push_back(CandPt);

		fprintf(fp, "%lf,%lf,0\n", x, y);
	}

	fclose(fp);

}

void lineFittingExample()
{
	vector<shared_ptr<Point2D>> points;
	createRandomLinePoints(points);

	ransac::RANSAC<Line2DModel, Point2D, 2> ransac_estimator;
	ransac_estimator.initialize(10, 1000, 0.3); // Threshold, iterations
	clock_t start = clock();
	ransac_estimator.estimate(points);
	int64_t end = clock();
	cout << "RANSAC took: " << (double)(end - start) / CLOCKS_PER_SEC * 1000.0 << " ms." << endl;

	FILE* fp2 = fopen("line_inlier_points.txt", "w");

	for (auto& i : ransac_estimator.getInlierIndices())
		fprintf(fp2, "%lf,%lf,0\n", points[i]->x, points[i]->y);

	fclose(fp2);

	FILE* fp3 = fopen("line.txt", "w");
	auto model = ransac_estimator.getBestModel();
	if (model)
	{
		std::vector<double> params;
		model->getParams(params);
		for (auto p : params)
			fprintf(fp3, "%lf\n", p);
	}
	fclose(fp3);
}

int main(int argc, char * argv[])
{
	planeFittingExample();

	lineFittingExample();

	getchar();
}