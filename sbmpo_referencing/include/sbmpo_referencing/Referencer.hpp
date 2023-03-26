#ifndef SBMPO_REFERENCER_HPP_
#define SBMPO_REFERENCER_HPP_

#include <sbmpo/sbmpo.hpp>

namespace sbmpo_referencing {

using namespace sbmpo;

template <typename ModelType>
class Referencer : public ModelType {
static_assert(std::is_base_of<sbmpo::Model, ModelType>::value, "ModelType must derive from sbmpo::Model");

    public:

    Referencer() {
        reference_factor_ = 1.0f;
    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) override {
        float gval = ModelType::cost(state, control, time_span);
        float jval = j_value(state);
        return gval + reference_factor_*jval;
    }

    // Compute j value
    virtual float j_value(const State& state) {
        if (reference_path_.empty())
            return 0.0f;
        float min_j = std::numeric_limits<float>::infinity();
        for (const State& ref_state : reference_path_) {
            float j = 0;
            // TODO
            if (j < min_j)
                min_j = j;
        }
        return min_j;
    }

    // Set the reference factor
    void set_reference_factor(float reference_factor) {
        reference_factor_ = reference_factor;
    }

    // Set the reference path
    void set_reference_path(std::vector<State> reference_path) {
        reference_path_ = reference_path;
    }

    private:

    float reference_factor_;
    std::vector<State> reference_path_;

};

}

#endif