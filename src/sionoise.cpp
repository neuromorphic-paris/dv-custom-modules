#define DV_API_OPENCV_SUPPORT 0
#include "dv-sdk/module.hpp"

#include <vector>


using matrixBufferT     = std::vector<uint32_t>;

class Sionoise : public dv::ModuleBase {
public:
	static const char *initDescription() {
        return "Noise filter using Sio's algorithm.";
    }

	static void initInputs(dv::InputDefinitionList &in) {
        in.addEventInput("events");
    }

	static void initOutputs(dv::OutputDefinitionList &out) {
        out.addEventOutput("events");
    }

	static void initConfigOptions(dv::RuntimeConfig &config) {
        config.add("threshold", dv::ConfigOption::intOption("Threshold value for timestamps.", 1, 1, 10000));  // ?
        config.add("size", dv::ConfigOption::intOption("Neighbourhood size (actually this*2+1).", 1, 1, 50));
        config.setPriorityOptions({"threshold", "size"});
    }

	Sionoise() {
        auto input   = inputs.getEventInput("events");
        sizeX        = input.sizeX();
        sizeY        = input.sizeY();
        matrixMem.resize(sizeX * sizeY);
        outputs.getEventOutput("events").setup(inputs.getEventInput("events"));
    }

	void run() override {
        auto inEvent  = inputs.getEventInput("events").events();
        auto outEvent = outputs.getEventOutput("events").events();

        if (!inEvent) {
            return;
        }

        for (auto &evt : inEvent) {

            // apply filter
            if (filterEvent(evt)) {
                outEvent << evt;
            }

            updateMatrix(evt);
        }
        outEvent << dv::commit;
    }

	void configUpdate() override {
        sz = config.getInt("size");
        threshold = static_cast<uint32_t>(config.getInt("threshold"));
    }

private:
	matrixBufferT matrixMem;
	uint32_t threshold;
	int sizeX;
	int sizeY;
	int sz;

	void updateMatrix(const dv::Event &event) {
        auto address                 = event.x() * sizeY + event.y();
        matrixMem[address] = static_cast<int32_t>(event.timestamp());
    }

	bool filterEvent(const dv::Event &event) {

        auto x = event.x();
	    auto y = event.y();
        auto t = static_cast<uint32_t>(event.timestamp());

        // boundary
        if (y < sz || y >= sizeY - sz || x < sz || x >= sizeX - sz) {
            return false;
        }

        for (int i = -sz; i <= sz; i++) {
            for (int j = -sz; j <= sz; j++) {
                if (i == 0 && j == 0) {
                    continue;
                }
                auto address = (event.x() + i) * sizeY + event.y() + j;
                if (t - matrixMem[address] < threshold) {
                    return true;
                }
            }
        }

        return false;
    }

};

registerModuleClass(Sionoise)