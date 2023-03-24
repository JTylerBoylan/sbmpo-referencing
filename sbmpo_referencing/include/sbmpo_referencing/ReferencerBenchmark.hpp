#ifndef SBMPO_REFERENCER_BENCHMARK_HPP_
#define SBMPO_REFERENCER_BENCHMARK_HPP_

#include <sbmpo_referencing/Referencer.hpp>
#include <sbmpo_benchmarking/benchmarks/Obstacles2D.hpp>

namespace sbmpo_referencing {

using namespace sbmpo_benchmarking;

template <typename ModelType>
class ReferencerBenchmark : public Referencer<ModelType>, public Obstacles2DBenchmark {
static_assert(std::is_base_of<sbmpo::Model, ModelType>::value, "ModelType must derive from sbmpo::Model");

    public:

    /// @brief Create a new Grid2D benchmark
    /// @param csv_folder Path to csv workspace folder
    ReferencerBenchmark(std::string csv_folder) : Referencer<ModelType>(), Obstacles2DBenchmark(csv_folder) 
    {
        body_radius_ = 0.25f;
        map_bounds_ = {-10.0f, -10.0f, 10.0f, 10.0f};
    }

    /// @brief Change the body radius (default 0.25)
    /// @param body_radius New body radius value
    void set_body_radius(float body_radius) {
        body_radius_ = body_radius;
    }

    /// @brief Change the benchmark map boundaries
    /// @param map_bounds Array of 4 boundary values ([xmin ymin xmax ymax])
    void set_map_bounds(std::array<float, 4> map_bounds) {
        map_bounds_ = map_bounds;
    }

    // Determine if node is valid (with obstacles and map bounds)
    bool is_valid(const State& state) override {
        
        // Bound check
        if (state[0] - map_bounds_[0] < body_radius_ ||
            state[1] - map_bounds_[1] < body_radius_ ||
            map_bounds_[2] - state[0] < body_radius_ ||
            map_bounds_[3] - state[1] < body_radius_)
            return false;

        // Obstacle collision check
        for (size_t o = 0; o < obstacles_.size(); o++) {
            const float dx = state[0] - obstacles_[o][0];
            const float dy = state[1] - obstacles_[o][1];
            const float threshold = obstacles_[o][2] + body_radius_;
            if (dx*dx + dy*dy < threshold*threshold)
                return false;
        }

        return true;
    }

        /// @brief Benchmark a model with 2D obstacles
    /// @param model Model to be benchmarked
    void benchmark(sbmpo::Model &model) override {

        csv_tool_.clear_results_file();

        size_t par = 0;
        std::vector<SBMPOParameters> parameters_list = csv_tool_.get_params();
        std::vector<Obstacles> obstaclesList = get_obstacles();
        for (auto param = parameters_list.begin(); param != parameters_list.end(); ++param) {

            if (verbose_) print_parameters(*param);

            int exit_code = UNKNOWN_ERROR;
            unsigned long time_us = 0.0;
            double cost = 0.0;
            int node_count = 0;
            int success_count = 0;
            int iterations = 0;

            obstacles_ = par < obstaclesList.size() ? obstaclesList[par++] : Obstacles();
            
            sbmpo::SBMPO sbmpo(model, *param);

            for (int r = 0; r < runs_per_param_; r++) {

                sbmpo.reset();
                sbmpo.run();

                exit_code = sbmpo.exit_code();
                time_us += sbmpo.time_us();
                cost += exit_code ? 0 : sbmpo.cost();
                iterations += sbmpo.iterations();
                node_count += sbmpo.size();
                if (!exit_code) success_count++;

            }

            unsigned long time_us_avg = time_us / runs_per_param_;
            float iterations_avg = double(iterations) / runs_per_param_;
            float cost_avg = cost / success_count;
            float node_count_avg = double(node_count) / runs_per_param_;
            float success_rate = double(success_count) / runs_per_param_;

            if (verbose_) print_stats(time_us_avg, exit_code, iterations_avg, cost_avg, node_count_avg, success_rate);
            if (verbose_) print_results(sbmpo);
            if (verbose_) print_obstacles(obstacles_);

            if (verbose_) printf("Writing results in folder %s ...\n", csv_tool_.get_save_folder().c_str());
            csv_tool_.append_stats(time_us_avg, exit_code, iterations_avg, cost_avg, node_count_avg, success_rate);
            csv_tool_.append_node_path(sbmpo.node_path(), sbmpo.control_path());
            csv_tool_.append_nodes(sbmpo.all_nodes());
            printf("\n");

            this->set_reference_path(sbmpo.state_path());

        }

        if (verbose_) printf("Finished benchmarking.\n");
    }

    protected:

    float body_radius_;
    std::array<float, 4> map_bounds_;

};

}

#endif