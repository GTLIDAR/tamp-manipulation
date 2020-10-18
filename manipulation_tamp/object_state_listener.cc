#include <iostream>

#include <gflags/gflags.h>
#include "lcm/lcm-cpp.hpp"
#include "drake/lcmt_combined_object_state.hpp"

namespace drake {
namespace manipulation_tamp {
namespace manipulation_station {

class ObjectStateListener {
    public:
    ObjectStateListener() {
        lcm_.subscribe("OBJECT_STATE", &ObjectStateListener::HandleObjectState, this);
    }

    void Run() {
        while (true) {
            while (lcm_.handleTimeout(10)==0) {}
        }
    }

    private:
    lcm::LCM lcm_;
    int64_t state_num_{0};

    void HandleObjectState(const lcm::ReceiveBuffer*, const std::string&,
                           const lcmt_combined_object_state* status) {
        state_num_++;

        if (state_num_%10==0) {
            std::cout<<"N_Objects: "<<status->num_objects<<"\n";
            std::cout<<"utime: "<<status->utime<<"\n";
            std::cout<<"q: ";
            for (int i = 0; i < status->q_dim; i++) {
                std::cout<< status->q[0][i] <<" ";
            }
            std::cout<<"\n";
            std::cout<<"v: ";
            for (int i = 0; i < status->v_dim; i++) {
                std::cout<< status->v[0][i] <<" ";
            }

        }
    }
};

}
}
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::manipulation_tamp::manipulation_station::ObjectStateListener listener;
    listener.Run();
    return 0;
}
