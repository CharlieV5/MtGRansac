#pragma once
#include <iostream>
#include <cmath>
#include <string>
#include <random>
#include <memory>
#include <algorithm>
#include <vector>
#include <omp.h>
#include <numeric>

namespace ransac
{
	// Each abstract model is made of abstract parameters
	// Could be anything from a point (that make a 2D line or 3D plane or image correspondences) to a line
	
	// Abstract model type for generic RANSAC model fitting

	template <class InputData>
	class Model
	{
	public:

		virtual void getParams(std::vector<double>& params) = 0;

		virtual double computeDistanceMeasure(std::shared_ptr<InputData> data) = 0;

		virtual void initModel(const std::vector<std::shared_ptr<InputData>> & v_data) = 0;

	};


	// M - Model
	template <class M, class InputData, int least_data_size>
	class RANSAC
	{
	public:
		RANSAC(void)
		{
			int thread_count = std::max(1, omp_get_max_threads());
			std::cout << "[ INFO ]: Maximum usable threads: " << thread_count << std::endl;
			for (int i = 0; i < thread_count; ++i)
			{
				std::random_device SeedDevice;
				m_rand_engines.push_back(std::mt19937(SeedDevice()));
			}

			m_probability = 0.8;
			m_threshold = 0.0;
			m_iterations = 1000;
		};

		virtual ~RANSAC(void) {};
		
		void initialize(double threshold, int max_iterations = 1000, double p = 0.8)
		{
			m_threshold = threshold;
			m_iterations = max_iterations;
			m_probability = p;
		};
		
		bool estimate(const std::vector<std::shared_ptr<InputData>> & v_data)
		{
			if (v_data.size() <= least_data_size)
			{
				std::cerr << "[ WARN ]: RANSAC - Number of data points is too less. Not doing anything." << std::endl;
				return false;
			}

			size_t data_size = v_data.size();
			int iterations = 0;
			double k = m_iterations;

			double log_probability = log(1.0 - m_probability);
			double one_over_indices = 1.0 / data_size;

			int thread_count = std::max(1, omp_get_max_threads());
			omp_set_dynamic(0); // Explicitly disable dynamic teams
			omp_set_num_threads(thread_count);
			size_t best_model_num = least_data_size;

			int iter_count = 0;
//#pragma omp parallel for
			for (int i = 0; i < m_iterations; ++i)
			{
				if (i >= k)
					break;

				// Select least_data_size random samples
				std::vector<std::shared_ptr<InputData>> random_samples(least_data_size);
								
				std::vector<int> random_indices(data_size);
				std::iota(random_indices.begin(), random_indices.end(), 0);
				std::shuffle(random_indices.begin(), random_indices.end(), m_rand_engines[omp_get_thread_num()]);
				// To avoid picking the same element more than once

				for (size_t k = 0; k < least_data_size; k++)
				{
					random_samples[k] = v_data.at(random_indices[k]);
				}

				std::shared_ptr<M> random_model(new M);
				random_model->initModel(random_samples);

				// Check if the sampled model is the best so far
				////////
				std::vector<size_t> indices;
				for (size_t index=0; index<data_size; ++index)
				{
					if (random_model->computeDistanceMeasure(v_data[index]) < m_threshold)
					{
						indices.push_back(index);
					}					
				}

				size_t maybe_inliers_num = indices.size();
				if (maybe_inliers_num > best_model_num)
				{
					best_model_num = maybe_inliers_num;
					m_inlier_indices = indices;
					m_best_model = random_model;
				}

				//recaculate k, k=log(1-p)/log(1-pow(w,n))
				double w = static_cast<double> (best_model_num) * one_over_indices;
				
				double p_no_outliers = 1.0 - std::pow(w, static_cast<double> (maybe_inliers_num));// This is the inlier fraction

				p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon(), p_no_outliers);       // Avoid division by -Inf
				p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);   // Avoid division by 0.
				k = log_probability / log(p_no_outliers);

				iter_count = i;
			}
			
			return true;
		};
		
		std::shared_ptr<M> getBestModel(void) { return m_best_model; };
		
		const std::vector<size_t>& getInlierIndices(void) { return m_inlier_indices; };
		
	private:				

		double m_probability;
		std::shared_ptr<M> m_best_model; // Pointer to the best model, valid only after estimate() is called
		std::vector<size_t> m_inlier_indices;

		int m_iterations; // Number of iterations before termination
		double m_threshold; // The threshold for computing model consensus

		std::vector<std::mt19937> m_rand_engines; // Mersenne twister high quality RNG that support *OpenMP* multi-threading

	};
} // namespace GRANSAC
