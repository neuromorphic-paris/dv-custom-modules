// Copyright 2020 iniVation AG
// Modified 2020 Lorenzo Vannucci
#define DV_API_OPENCV_SUPPORT 0

#include "dv-sdk/cross/portable_time.h"
#include "dv-sdk/data/event.hpp"
#include "dv-sdk/data/frame.hpp"
#include "dv-sdk/data/imu.hpp"
#include "dv-sdk/data/trigger.hpp"
#include "dv-sdk/module.hpp"

// #include "dv-sdk/log.hpp"
#include "log.hpp"
#include "aedat4_convert.hpp"

#include <libcaercpp/devices/davis.hpp>

class davis : public dv::ModuleBase {
private:
	libcaer::devices::davis device;

public:
	static void initOutputs(dv::OutputDefinitionList &out) {
		out.addEventOutput("events");
		out.addFrameOutput("frames");
		out.addTriggerOutput("triggers");
		out.addIMUOutput("imu");
	}

	static const char *initDescription() {
		return ("iniVation DAVIS camera support, with synchronization option.");
	}

	static void initConfigOptions(dv::RuntimeConfig &config) {
		config.add("busNumber", dv::ConfigOption::intOption("USB bus number restriction.", 0, 0, UINT8_MAX));
		config.add("devAddress", dv::ConfigOption::intOption("USB device address restriction.", 0, 0, UINT8_MAX));
		config.add("serialNumber", dv::ConfigOption::stringOption("USB serial number restriction.", ""));

		config.add("initialized", dv::ConfigOption::boolOption("sync event received", false, true));
		config.add("resetInitialization", dv::ConfigOption::buttonOption("Resets the initialization state", "Reset init state"));

		config.setPriorityOptions({"dataMode", "initialized", "resetInitialization"});

		multiplexerConfigCreate(config);
		dvsConfigCreate(config);
		apsConfigCreate(config);
		imuConfigCreate(config);
		externalInputConfigCreate(config);
		usbConfigCreate(config);
		systemConfigCreate(config);
	}

	davis() :
		device(0, static_cast<uint8_t>(config.getInt("busNumber")), static_cast<uint8_t>(config.getInt("devAddress")),
			config.getString("serialNumber")) {
		// Initialize per-device log-level to module log-level.
		device.configSet(CAER_HOST_CONFIG_LOG, CAER_HOST_CONFIG_LOG_LEVEL,
			static_cast<uint32_t>(dv::LoggerInternal::logLevelNameToInteger(config.getString("logLevel"))));

		auto devInfo = device.infoGet();

		// Generate source string for output modules.
		auto sourceString = chipIDToName(devInfo.chipID, false) + "_" + devInfo.deviceSerialNumber;

		// Setup outputs.
		outputs.getEventOutput("events").setup(device.infoGet().dvsSizeX, device.infoGet().dvsSizeY, sourceString);
		outputs.getEventOutput("frames").setup(device.infoGet().apsSizeX, device.infoGet().apsSizeY, sourceString);
		outputs.getTriggerOutput("triggers").setup(sourceString);
		outputs.getIMUOutput("imu").setup(sourceString);

		auto sourceInfoNode = moduleNode.getRelativeNode("sourceInfo/");

		sourceInfoNode.create<dv::CfgType::STRING>("serialNumber", devInfo.deviceSerialNumber, {0, 8},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device serial number.");
		sourceInfoNode.create<dv::CfgType::INT>("usbBusNumber", devInfo.deviceUSBBusNumber, {0, 255},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device USB bus number.");
		sourceInfoNode.create<dv::CfgType::INT>("usbDeviceAddress", devInfo.deviceUSBDeviceAddress, {0, 255},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device USB device address.");

		sourceInfoNode.create<dv::CfgType::INT>("firmwareVersion", devInfo.firmwareVersion,
			{devInfo.firmwareVersion, devInfo.firmwareVersion}, dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
			"Device USB firmware version.");
		sourceInfoNode.create<dv::CfgType::INT>("logicVersion", devInfo.logicVersion,
			{devInfo.logicVersion, devInfo.logicVersion}, dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
			"Device FPGA logic version.");
		sourceInfoNode.create<dv::CfgType::INT>("chipID", devInfo.chipID, {devInfo.chipID, devInfo.chipID},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device chip identification number.");

		// Extra features.
		sourceInfoNode.create<dv::CfgType::BOOL>("muxHasStatistics", devInfo.muxHasStatistics, {},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
			"Device supports FPGA Multiplexer statistics (USB event drops).");

		sourceInfoNode.create<dv::CfgType::BOOL>("dvsHasPixelFilter", devInfo.dvsHasPixelFilter, {},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device supports FPGA DVS Pixel-level filter.");
		sourceInfoNode.create<dv::CfgType::BOOL>("dvsHasNoiseAndRateFilter", devInfo.dvsHasBackgroundActivityFilter, {},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device supports FPGA DVS Noise and Rate filter.");
		sourceInfoNode.create<dv::CfgType::BOOL>("dvsHasROIFilter", devInfo.dvsHasROIFilter, {},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device supports FPGA DVS ROI filter.");
		sourceInfoNode.create<dv::CfgType::BOOL>("dvsHasSkipFilter", devInfo.dvsHasSkipFilter, {},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device supports FPGA DVS skip events filter.");
		sourceInfoNode.create<dv::CfgType::BOOL>("dvsHasPolarityFilter", devInfo.dvsHasPolarityFilter, {},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device supports FPGA DVS polarity filter.");
		sourceInfoNode.create<dv::CfgType::BOOL>("dvsHasStatistics", devInfo.dvsHasStatistics, {},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device supports FPGA DVS statistics.");

		sourceInfoNode.create<dv::CfgType::INT>("apsColorFilter", devInfo.apsColorFilter,
			{devInfo.apsColorFilter, devInfo.apsColorFilter}, dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
			"APS sensor color-filter pattern.");
		sourceInfoNode.create<dv::CfgType::BOOL>("apsHasGlobalShutter", devInfo.apsHasGlobalShutter, {},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "APS sensor supports global-shutter mode.");

		// Add color filter information to event output too.
		if (devInfo.apsColorFilter) {
			outputs.getEventOutput("events").infoNode().create<dv::CfgType::INT>("colorFilter",
				(devInfo.apsColorFilter - 1) & 0x03, {0, 3}, dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
				"Sensor color-filter pattern.");
		}

		sourceInfoNode.create<dv::CfgType::BOOL>("extInputHasGenerator", devInfo.extInputHasGenerator, {},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
			"Device supports generating pulses on output signal connector.");

		sourceInfoNode.create<dv::CfgType::BOOL>("deviceIsMaster", devInfo.deviceIsMaster, {},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
			"Timestamp synchronization support: device master status.");

		sourceInfoNode.create<dv::CfgType::STRING>("source", sourceString,
			{static_cast<int32_t>(sourceString.length()), static_cast<int32_t>(sourceString.length())},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT, "Device source information.");

		// Ensure good defaults for data acquisition settings.
		// No blocking behavior due to mainloop notification, and no auto-start of
		// all producers to ensure cAER settings are respected.
		device.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
		device.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS, false);
		device.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS, true);

		// DVS240 supports only either Events or Frames. We use the
		// IMU Type field to recognize new generation devices.
		if (IS_DAVIS240C(devInfo.chipID) && (devInfo.imuType == IMU_INVENSENSE_9250)) {
			config.add(
				"dataMode", dv::ConfigOption::listOption("Camera data mode.", 0, {"Events only", "Frames only"}));

			if (config.getString("dataMode") == "Events+Frames") {
				config.setString("dataMode", "Events only");
			}
		}
		else {
			config.add("dataMode",
				dv::ConfigOption::listOption("Camera data mode.", 0, {"Events+Frames", "Events only", "Frames only"}));
		}

		// Create default device-dependant settings.
		biasConfigCreateDynamic(&devInfo);
		chipConfigCreateDynamic(&devInfo);
		multiplexerConfigCreateDynamic(&devInfo);
		dvsConfigCreateDynamic(&devInfo);
		apsConfigCreateDynamic(&devInfo);
		imuConfigCreateDynamic(&devInfo);
		externalInputConfigCreateDynamic(&devInfo);

		// Set timestamp offset for real-time timestamps. DataStart() will
		// reset the device-side timestamp.
		struct timespec tsNow;
		portable_clock_gettime_realtime(&tsNow);

		int64_t tsNowOffset
			= static_cast<int64_t>(tsNow.tv_sec * 1000000LL) + static_cast<int64_t>(tsNow.tv_nsec / 1000LL);

		sourceInfoNode.create<dv::CfgType::LONG>("tsOffset", tsNowOffset, {0, INT64_MAX},
			dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
			"Time offset of data stream starting point to Unix time in µs.");

		moduleNode.getRelativeNode("outputs/events/info/")
			.create<dv::CfgType::LONG>("tsOffset", tsNowOffset, {0, INT64_MAX},
				dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
				"Time offset of data stream starting point to Unix time in µs.");

		moduleNode.getRelativeNode("outputs/frames/info/")
			.create<dv::CfgType::LONG>("tsOffset", tsNowOffset, {0, INT64_MAX},
				dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
				"Time offset of data stream starting point to Unix time in µs.");

		moduleNode.getRelativeNode("outputs/triggers/info/")
			.create<dv::CfgType::LONG>("tsOffset", tsNowOffset, {0, INT64_MAX},
				dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
				"Time offset of data stream starting point to Unix time in µs.");

		moduleNode.getRelativeNode("outputs/imu/info/")
			.create<dv::CfgType::LONG>("tsOffset", tsNowOffset, {0, INT64_MAX},
				dv::CfgFlags::READ_ONLY | dv::CfgFlags::NO_EXPORT,
				"Time offset of data stream starting point to Unix time in µs.");

		// Start data acquisition.
		device.dataStart(nullptr, nullptr, nullptr, &moduleShutdownNotify, moduleData->moduleNode);

		// Send all configuration to the device.
		sendDefaultConfiguration(&devInfo);

		// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
		moduleNode.getRelativeNode("multiplexer/").addAttributeListener(&device, &multiplexerConfigListener);

		moduleNode.getRelativeNode("dvs/").addAttributeListener(&device, &dvsConfigListener);

		for (auto &dvsFilter : moduleNode.getRelativeNode("dvs/").getChildren()) {
			dvsFilter.addAttributeListener(&device, &dvsConfigListener);
		}

		moduleNode.getRelativeNode("aps/").addAttributeListener(&device, &apsConfigListener);

		moduleNode.getRelativeNode("imu/").addAttributeListener(&device, &imuConfigListener);

		moduleNode.getRelativeNode("externalInput/").addAttributeListener(&device, &externalInputConfigListener);

		moduleNode.getRelativeNode("usb/").addAttributeListener(&device, &usbConfigListener);

		moduleNode.getRelativeNode("system/").addAttributeListener(&device, &systemConfigListener);

		moduleNode.addAttributeListener(&device, &logLevelListener);

		moduleNode.addAttributeListener(&device, &modeListener);

		auto chipNode = moduleNode.getRelativeNode(chipIDToName(devInfo.chipID, true));

		chipNode.getRelativeNode("chip/").addAttributeListener(&device, &chipConfigListener);

		auto biasNode = chipNode.getRelativeNode("bias/");

		for (auto &singleBias : biasNode.getChildren()) {
			singleBias.addAttributeListener(&device, &biasConfigListener);
		}
	}

	~davis() override {
		auto devInfo = device.infoGet();

		// Remove listener, which can reference invalid memory in userData.
		moduleNode.getRelativeNode("multiplexer/").removeAttributeListener(&device, &multiplexerConfigListener);

		moduleNode.getRelativeNode("dvs/").removeAttributeListener(&device, &dvsConfigListener);

		for (auto &dvsFilter : moduleNode.getRelativeNode("dvs/").getChildren()) {
			dvsFilter.removeAttributeListener(&device, &dvsConfigListener);
		}

		moduleNode.getRelativeNode("aps/").removeAttributeListener(&device, &apsConfigListener);

		moduleNode.getRelativeNode("imu/").removeAttributeListener(&device, &imuConfigListener);

		moduleNode.getRelativeNode("externalInput/").removeAttributeListener(&device, &externalInputConfigListener);

		moduleNode.getRelativeNode("usb/").removeAttributeListener(&device, &usbConfigListener);

		moduleNode.getRelativeNode("system/").removeAttributeListener(&device, &systemConfigListener);

		moduleNode.removeAttributeListener(&device, &logLevelListener);

		moduleNode.removeAttributeListener(&device, &modeListener);

		auto chipNode = moduleNode.getRelativeNode(chipIDToName(devInfo.chipID, true));

		chipNode.getRelativeNode("chip/").removeAttributeListener(&device, &chipConfigListener);

		auto biasNode = chipNode.getRelativeNode("bias/");

		for (auto &singleBias : biasNode.getChildren()) {
			singleBias.removeAttributeListener(&device, &biasConfigListener);
		}

		// Stop data acquisition.
		device.dataStop();

		// Ensure Exposure value is coherent with libcaer.
		moduleNode.getRelativeNode("aps/").attributeUpdaterRemoveAll();
		moduleNode.getRelativeNode("aps/").putInt(
			"Exposure", apsExposureUpdater(&device, "Exposure", DVCFG_TYPE_INT).iint);

		// Remove statistics read modifiers.
		if (moduleNode.existsRelativeNode("statistics/")) {
			moduleNode.getRelativeNode("statistics/").attributeUpdaterRemoveAll();
		}

		// Clear sourceInfo node.
		auto sourceInfoNode = moduleNode.getRelativeNode("sourceInfo/");
		sourceInfoNode.removeAllAttributes();
	}

	void run() override {

		if (config.getBool("resetInitialization")) {
			config.setBool("resetInitialization", false);
			config.setBool("initialized", false);
		}

		auto data = device.dataGet();

		if (!data || data->empty()) {
			return;
		}

		if (data->getEventPacket(SPECIAL_EVENT)) {
			std::shared_ptr<const libcaer::events::SpecialEventPacket> special
				= std::static_pointer_cast<libcaer::events::SpecialEventPacket>(data->getEventPacket(SPECIAL_EVENT));

			if (special->getEventNumber() == 1 && (*special)[0].getType() == TIMESTAMP_RESET) {

				config.setBool("initialized", true);

				// Update master/slave information.
				auto devInfo = device.infoGet();

				auto sourceInfoNode = moduleNode.getRelativeNode("sourceInfo/");
				sourceInfoNode.updateReadOnly<dv::CfgType::BOOL>("deviceIsMaster", devInfo.deviceIsMaster);

				// Reset real-time timestamp offset.
				struct timespec tsNow;
				portable_clock_gettime_realtime(&tsNow);

				int64_t tsNowOffset
					= static_cast<int64_t>(tsNow.tv_sec * 1000000LL) + static_cast<int64_t>(tsNow.tv_nsec / 1000LL);

				sourceInfoNode.updateReadOnly<dv::CfgType::LONG>("tsOffset", tsNowOffset);

				moduleNode.getRelativeNode("outputs/events/info/")
					.updateReadOnly<dv::CfgType::LONG>("tsOffset", tsNowOffset);

				moduleNode.getRelativeNode("outputs/frames/info/")
					.updateReadOnly<dv::CfgType::LONG>("tsOffset", tsNowOffset);

				moduleNode.getRelativeNode("outputs/triggers/info/")
					.updateReadOnly<dv::CfgType::LONG>("tsOffset", tsNowOffset);

				moduleNode.getRelativeNode("outputs/imu/info/")
					.updateReadOnly<dv::CfgType::LONG>("tsOffset", tsNowOffset);
			}

			dvConvertToAedat4(special->getHeaderPointer(), moduleData);
		}

		if (data->size() == 1) {
			return;
		}

		if (!config.getBool("initialized")) {
			return;
		}

		if (data->getEventPacket(POLARITY_EVENT)) {
			dvConvertToAedat4(data->getEventPacket(POLARITY_EVENT)->getHeaderPointer(), moduleData);
		}

		if (data->getEventPacket(FRAME_EVENT)) {
			dvConvertToAedat4(data->getEventPacket(FRAME_EVENT)->getHeaderPointer(), moduleData);
		}

		if (data->getEventPacket(IMU6_EVENT)) {
			dvConvertToAedat4(data->getEventPacket(IMU6_EVENT)->getHeaderPointer(), moduleData);
		}
	}

private:
	static void moduleShutdownNotify(void *p) {
		dv::Cfg::Node moduleNode = static_cast<dvConfigNode>(p);

		// Ensure parent also shuts down (on disconnected device for example).
		moduleNode.putBool("running", false);
	}

	static inline std::string chipIDToName(int16_t chipID, bool withEndSlash) {
		std::string result;

		switch (chipID) {
			case 0:
				result = "DAVIS240A";
				break;

			case 1:
				result = "DAVIS240B";
				break;

			case 2:
				result = "DAVIS240C";
				break;

			case 3:
				result = "DAVIS128";
				break;

			case 5: // DAVIS346B -> only FSI chip.
				result = "DAVIS346";
				break;

			case 6:
				result = "DAVIS640";
				break;

			case 7:
				result = "DAVIS640H";
				break;

			case 8: // PixelParade.
				result = "DAVIS208";
				break;

			case 9: // DAVIS346Cbsi -> only BSI chip.
				result = "DAVIS346BSI";
				break;

			default:
				result = "Unsupported";
				break;
		}

		if (withEndSlash) {
			result += "/";
		}

		return (result);
	}

	void sendDefaultConfiguration(const struct caer_davis_info *devInfo) {
		// Send cAER configuration to libcaer and device.
		biasConfigSend(devInfo);
		chipConfigSend(devInfo);

		// Wait 200 ms for biases to stabilize.
		struct timespec biasEnSleep = {.tv_sec = 0, .tv_nsec = 200000000};
		nanosleep(&biasEnSleep, nullptr);

		systemConfigSend();
		usbConfigSend();
		multiplexerConfigSend();

		// Wait 50 ms for data transfer to be ready.
		struct timespec noDataSleep = {.tv_sec = 0, .tv_nsec = 50000000};
		nanosleep(&noDataSleep, nullptr);

		dvsConfigSend(devInfo);
		apsConfigSend(devInfo);
		imuConfigSend(devInfo);
		externalInputConfigSend(devInfo);
	}

	void biasConfigCreateDynamic(const struct caer_davis_info *devInfo) {
		auto biasPath = chipIDToName(devInfo->chipID, true) + "bias/";

		if (IS_DAVIS240(devInfo->chipID)) {
			createCoarseFineBiasSetting(biasPath + "DiffBn", 4, 39, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "OnBn", 5, 255, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "OffBn", 4, 0, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "ApsCasEpc", 5, 185, true, "N", "Cascode");
			createCoarseFineBiasSetting(biasPath + "DiffCasBnc", 5, 115, true, "N", "Cascode");
			createCoarseFineBiasSetting(biasPath + "ApsROSFBn", 6, 219, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "LocalBufBn", 5, 164, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "PixInvBn", 6, 144, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "PrBp", 2, 58, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "PrSFBp", 1, 16, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "RefrBp", 4, 25, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "AEPdBn", 6, 91, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "LcolTimeoutBn", 5, 49, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "AEPuXBp", 4, 80, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "AEPuYBp", 7, 152, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "IFThrBn", 5, 255, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "IFRefrBn", 5, 255, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "PadFollBn", 7, 215, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "ApsOverflowLevelBn", 6, 253, true, "N", "Normal");

			createCoarseFineBiasSetting(biasPath + "BiasBuffer", 5, 254, true, "N", "Normal");

			createShiftedSourceBiasSetting(biasPath + "SSP", 1, 33, "ShiftedSource", "SplitGate");
			createShiftedSourceBiasSetting(biasPath + "SSN", 1, 33, "ShiftedSource", "SplitGate");
		}

		if (IS_DAVIS128(devInfo->chipID) || IS_DAVIS208(devInfo->chipID) || IS_DAVIS346(devInfo->chipID)
			|| IS_DAVIS640(devInfo->chipID)) {
			// This is first so that it takes precedence over later settings for all other chips.
			if (IS_DAVIS640(devInfo->chipID)) {
				// Slow down pixels for big 640x480 array, to avoid overwhelming the AER bus.
				createCoarseFineBiasSetting(biasPath + "PrBp", 2, 3, true, "P", "Normal");
				createCoarseFineBiasSetting(biasPath + "PrSFBp", 1, 1, true, "P", "Normal");
				createCoarseFineBiasSetting(biasPath + "OnBn", 5, 155, true, "N", "Normal");
				createCoarseFineBiasSetting(biasPath + "OffBn", 1, 4, true, "N", "Normal");

				createCoarseFineBiasSetting(biasPath + "BiasBuffer", 6, 125, true, "N", "Normal");
			}

			createVDACBiasSetting(biasPath + "ApsOverflowLevel", 27, 6);
			createVDACBiasSetting(biasPath + "ApsCas", 21, 6);
			createVDACBiasSetting(biasPath + "AdcRefHigh", 32, 7);
			createVDACBiasSetting(biasPath + "AdcRefLow", 1, 7);

			if (IS_DAVIS346(devInfo->chipID) || IS_DAVIS640(devInfo->chipID)) {
				// Only DAVIS346 and 640 have ADC testing.
				createVDACBiasSetting(biasPath + "AdcTestVoltage", 21, 7);
			}

			if (IS_DAVIS208(devInfo->chipID)) {
				createVDACBiasSetting(biasPath + "ResetHighPass", 63, 7);
				createVDACBiasSetting(biasPath + "RefSS", 11, 5);

				createCoarseFineBiasSetting(biasPath + "RegBiasBp", 5, 20, true, "P", "Normal");
				createCoarseFineBiasSetting(biasPath + "RefSSBn", 5, 20, true, "N", "Normal");
			}

			createCoarseFineBiasSetting(biasPath + "LocalBufBn", 5, 164, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "PadFollBn", 7, 215, false, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "DiffBn", 4, 39, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "OnBn", 5, 255, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "OffBn", 4, 1, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "PixInvBn", 6, 144, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "PrBp", 2, 58, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "PrSFBp", 1, 16, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "RefrBp", 4, 25, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "ReadoutBufBp", 6, 20, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "ApsROSFBn", 6, 219, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "AdcCompBp", 5, 20, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "ColSelLowBn", 0, 1, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "DACBufBp", 6, 60, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "LcolTimeoutBn", 5, 49, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "AEPdBn", 6, 91, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "AEPuXBp", 4, 80, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "AEPuYBp", 7, 152, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "IFRefrBn", 5, 255, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "IFThrBn", 5, 255, true, "N", "Normal");

			createCoarseFineBiasSetting(biasPath + "BiasBuffer", 5, 254, true, "N", "Normal");

			createShiftedSourceBiasSetting(biasPath + "SSP", 1, 33, "ShiftedSource", "SplitGate");
			createShiftedSourceBiasSetting(biasPath + "SSN", 1, 33, "ShiftedSource", "SplitGate");
		}

		if (IS_DAVIS640H(devInfo->chipID)) {
			createVDACBiasSetting(biasPath + "ApsCas", 21, 4);
			createVDACBiasSetting(biasPath + "OVG1Lo", 63, 4);
			createVDACBiasSetting(biasPath + "OVG2Lo", 0, 0);
			createVDACBiasSetting(biasPath + "TX2OVG2Hi", 63, 0);
			createVDACBiasSetting(biasPath + "Gnd07", 13, 4);
			createVDACBiasSetting(biasPath + "AdcTestVoltage", 21, 0);
			createVDACBiasSetting(biasPath + "AdcRefHigh", 46, 7);
			createVDACBiasSetting(biasPath + "AdcRefLow", 3, 7);

			createCoarseFineBiasSetting(biasPath + "IFRefrBn", 5, 255, false, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "IFThrBn", 5, 255, false, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "LocalBufBn", 5, 164, false, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "PadFollBn", 7, 209, false, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "PixInvBn", 4, 164, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "DiffBn", 3, 75, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "OnBn", 6, 95, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "OffBn", 2, 41, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "PrBp", 1, 88, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "PrSFBp", 1, 173, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "RefrBp", 2, 62, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "ArrayBiasBufferBn", 6, 128, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "ArrayLogicBufferBn", 5, 255, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "FalltimeBn", 7, 41, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "RisetimeBp", 6, 162, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "ReadoutBufBp", 6, 20, false, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "ApsROSFBn", 7, 82, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "AdcCompBp", 4, 159, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "DACBufBp", 6, 194, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "LcolTimeoutBn", 5, 49, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "AEPdBn", 6, 91, true, "N", "Normal");
			createCoarseFineBiasSetting(biasPath + "AEPuXBp", 4, 80, true, "P", "Normal");
			createCoarseFineBiasSetting(biasPath + "AEPuYBp", 7, 152, true, "P", "Normal");

			createCoarseFineBiasSetting(biasPath + "BiasBuffer", 6, 251, true, "N", "Normal");

			createShiftedSourceBiasSetting(biasPath + "SSP", 1, 33, "TiedToRail", "SplitGate");
			createShiftedSourceBiasSetting(biasPath + "SSN", 2, 33, "ShiftedSource", "SplitGate");
		}
	}

	void biasConfigSend(const struct caer_davis_info *devInfo) {
		auto biasPath = chipIDToName(devInfo->chipID, true) + "bias/";

		// All chips of a kind have the same bias address for the same bias!
		if (IS_DAVIS240(devInfo->chipID)) {
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFBN, generateCoarseFineBias(biasPath + "DiffBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_ONBN, generateCoarseFineBias(biasPath + "OnBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_OFFBN, generateCoarseFineBias(biasPath + "OffBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSCASEPC, generateCoarseFineBias(biasPath + "ApsCasEpc"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFCASBNC, generateCoarseFineBias(biasPath + "DiffCasBnc"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSROSFBN, generateCoarseFineBias(biasPath + "ApsROSFBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_LOCALBUFBN, generateCoarseFineBias(biasPath + "LocalBufBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PIXINVBN, generateCoarseFineBias(biasPath + "PixInvBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, generateCoarseFineBias(biasPath + "PrBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, generateCoarseFineBias(biasPath + "PrSFBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_REFRBP, generateCoarseFineBias(biasPath + "RefrBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPDBN, generateCoarseFineBias(biasPath + "AEPdBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_LCOLTIMEOUTBN,
				generateCoarseFineBias(biasPath + "LcolTimeoutBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPUXBP, generateCoarseFineBias(biasPath + "AEPuXBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPUYBP, generateCoarseFineBias(biasPath + "AEPuYBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_IFTHRBN, generateCoarseFineBias(biasPath + "IFThrBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_IFREFRBN, generateCoarseFineBias(biasPath + "IFRefrBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PADFOLLBN, generateCoarseFineBias(biasPath + "PadFollBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSOVERFLOWLEVELBN,
				generateCoarseFineBias(biasPath + "ApsOverflowLevelBn"));

			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_BIASBUFFER, generateCoarseFineBias(biasPath + "BiasBuffer"));

			device.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_SSP, generateShiftedSourceBias(biasPath + "SSP"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_SSN, generateShiftedSourceBias(biasPath + "SSN"));
		}

		if (IS_DAVIS128(devInfo->chipID) || IS_DAVIS208(devInfo->chipID) || IS_DAVIS346(devInfo->chipID)
			|| IS_DAVIS640(devInfo->chipID)) {
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSOVERFLOWLEVEL,
				generateVDACBias(biasPath + "ApsOverflowLevel"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSCAS, generateVDACBias(biasPath + "ApsCas"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCREFHIGH, generateVDACBias(biasPath + "AdcRefHigh"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCREFLOW, generateVDACBias(biasPath + "AdcRefLow"));

			if (IS_DAVIS346(devInfo->chipID) || IS_DAVIS640(devInfo->chipID)) {
				device.configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE,
					generateVDACBias(biasPath + "AdcTestVoltage"));
			}

			if (IS_DAVIS208(devInfo->chipID)) {
				device.configSet(DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_RESETHIGHPASS,
					generateVDACBias(biasPath + "ResetHighPass"));
				device.configSet(DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REFSS, generateVDACBias(biasPath + "RefSS"));

				device.configSet(
					DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REGBIASBP, generateCoarseFineBias(biasPath + "RegBiasBp"));
				device.configSet(
					DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REFSSBN, generateCoarseFineBias(biasPath + "RefSSBn"));
			}

			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_LOCALBUFBN, generateCoarseFineBias(biasPath + "LocalBufBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PADFOLLBN, generateCoarseFineBias(biasPath + "PadFollBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_DIFFBN, generateCoarseFineBias(biasPath + "DiffBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ONBN, generateCoarseFineBias(biasPath + "OnBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_OFFBN, generateCoarseFineBias(biasPath + "OffBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PIXINVBN, generateCoarseFineBias(biasPath + "PixInvBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PRBP, generateCoarseFineBias(biasPath + "PrBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PRSFBP, generateCoarseFineBias(biasPath + "PrSFBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_REFRBP, generateCoarseFineBias(biasPath + "RefrBp"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_READOUTBUFBP,
				generateCoarseFineBias(biasPath + "ReadoutBufBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSROSFBN, generateCoarseFineBias(biasPath + "ApsROSFBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCCOMPBP, generateCoarseFineBias(biasPath + "AdcCompBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_COLSELLOWBN, generateCoarseFineBias(biasPath + "ColSelLowBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_DACBUFBP, generateCoarseFineBias(biasPath + "DACBufBp"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_LCOLTIMEOUTBN,
				generateCoarseFineBias(biasPath + "LcolTimeoutBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPDBN, generateCoarseFineBias(biasPath + "AEPdBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPUXBP, generateCoarseFineBias(biasPath + "AEPuXBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPUYBP, generateCoarseFineBias(biasPath + "AEPuYBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_IFREFRBN, generateCoarseFineBias(biasPath + "IFRefrBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_IFTHRBN, generateCoarseFineBias(biasPath + "IFThrBn"));

			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_BIASBUFFER, generateCoarseFineBias(biasPath + "BiasBuffer"));

			device.configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_SSP, generateShiftedSourceBias(biasPath + "SSP"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_SSN, generateShiftedSourceBias(biasPath + "SSN"));
		}

		if (IS_DAVIS640H(devInfo->chipID)) {
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_APSCAS, generateVDACBias(biasPath + "ApsCas"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_OVG1LO, generateVDACBias(biasPath + "OVG1Lo"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_OVG2LO, generateVDACBias(biasPath + "OVG2Lo"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_TX2OVG2HI, generateVDACBias(biasPath + "TX2OVG2Hi"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_GND07, generateVDACBias(biasPath + "Gnd07"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCTESTVOLTAGE, generateVDACBias(biasPath + "AdcTestVoltage"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCREFHIGH, generateVDACBias(biasPath + "AdcRefHigh"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCREFLOW, generateVDACBias(biasPath + "AdcRefLow"));

			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_IFREFRBN, generateCoarseFineBias(biasPath + "IFRefrBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_IFTHRBN, generateCoarseFineBias(biasPath + "IFThrBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_LOCALBUFBN, generateCoarseFineBias(biasPath + "LocalBufBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PADFOLLBN, generateCoarseFineBias(biasPath + "PadFollBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PIXINVBN, generateCoarseFineBias(biasPath + "PixInvBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_DIFFBN, generateCoarseFineBias(biasPath + "DiffBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ONBN, generateCoarseFineBias(biasPath + "OnBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_OFFBN, generateCoarseFineBias(biasPath + "OffBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PRBP, generateCoarseFineBias(biasPath + "PrBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PRSFBP, generateCoarseFineBias(biasPath + "PrSFBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_REFRBP, generateCoarseFineBias(biasPath + "RefrBp"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ARRAYBIASBUFFERBN,
				generateCoarseFineBias(biasPath + "ArrayBiasBufferBn"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ARRAYLOGICBUFFERBN,
				generateCoarseFineBias(biasPath + "ArrayLogicBufferBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_FALLTIMEBN, generateCoarseFineBias(biasPath + "FalltimeBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_RISETIMEBP, generateCoarseFineBias(biasPath + "RisetimeBp"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_READOUTBUFBP,
				generateCoarseFineBias(biasPath + "ReadoutBufBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_APSROSFBN, generateCoarseFineBias(biasPath + "ApsROSFBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCCOMPBP, generateCoarseFineBias(biasPath + "AdcCompBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_DACBUFBP, generateCoarseFineBias(biasPath + "DACBufBp"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_LCOLTIMEOUTBN,
				generateCoarseFineBias(biasPath + "LcolTimeoutBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_AEPDBN, generateCoarseFineBias(biasPath + "AEPdBn"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_AEPUXBP, generateCoarseFineBias(biasPath + "AEPuXBp"));
			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_AEPUYBP, generateCoarseFineBias(biasPath + "AEPuYBp"));

			device.configSet(
				DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_BIASBUFFER, generateCoarseFineBias(biasPath + "BiasBuffer"));

			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_SSP, generateShiftedSourceBias(biasPath + "SSP"));
			device.configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_SSN, generateShiftedSourceBias(biasPath + "SSN"));
		}
	}

	static void biasConfigListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		UNUSED_ARGUMENT(changeKey);
		UNUSED_ARGUMENT(changeType);
		UNUSED_ARGUMENT(changeValue);

		auto device         = static_cast<libcaer::devices::davis *>(userData);
		const auto devInfo  = device->infoGet();
		const auto nodeName = dv::Cfg::Node(node).getName();

		if (event == DVCFG_ATTRIBUTE_MODIFIED) {
			if (IS_DAVIS240(devInfo.chipID)) {
				if (nodeName == "DiffBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "OnBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_ONBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "OffBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_OFFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "ApsCasEpc") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSCASEPC, generateCoarseFineBias(node));
				}
				else if (nodeName == "DiffCasBnc") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFCASBNC, generateCoarseFineBias(node));
				}
				else if (nodeName == "ApsROSFBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSROSFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "LocalBufBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_LOCALBUFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "PixInvBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PIXINVBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "PrBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "PrSFBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "RefrBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_REFRBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "AEPdBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPDBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "LcolTimeoutBn") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_LCOLTIMEOUTBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "AEPuXBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPUXBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "AEPuYBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPUYBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "IFThrBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_IFTHRBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "IFRefrBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_IFREFRBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "PadFollBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PADFOLLBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "ApsOverflowLevelBn") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSOVERFLOWLEVELBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "BiasBuffer") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_BIASBUFFER, generateCoarseFineBias(node));
				}
				else if (nodeName == "SSP") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_SSP, generateShiftedSourceBias(node));
				}
				else if (nodeName == "SSP") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_SSN, generateShiftedSourceBias(node));
				}
			}

			if (IS_DAVIS128(devInfo.chipID) || IS_DAVIS208(devInfo.chipID) || IS_DAVIS346(devInfo.chipID)
				|| IS_DAVIS640(devInfo.chipID)) {
				if (nodeName == "ApsOverflowLevel") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSOVERFLOWLEVEL, generateVDACBias(node));
				}
				else if (nodeName == "ApsCas") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSCAS, generateVDACBias(node));
				}
				else if (nodeName == "AdcRefHigh") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCREFHIGH, generateVDACBias(node));
				}
				else if (nodeName == "AdcRefLow") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCREFLOW, generateVDACBias(node));
				}
				else if ((IS_DAVIS346(devInfo.chipID) || IS_DAVIS640(devInfo.chipID)) && nodeName == "AdcTestVoltage") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE, generateVDACBias(node));
				}
				else if ((IS_DAVIS208(devInfo.chipID)) && nodeName == "ResetHighPass") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_RESETHIGHPASS, generateVDACBias(node));
				}
				else if ((IS_DAVIS208(devInfo.chipID)) && nodeName == "RefSS") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REFSS, generateVDACBias(node));
				}
				else if ((IS_DAVIS208(devInfo.chipID)) && nodeName == "RegBiasBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REGBIASBP, generateCoarseFineBias(node));
				}
				else if ((IS_DAVIS208(devInfo.chipID)) && nodeName == "RefSSBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REFSSBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "LocalBufBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_LOCALBUFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "PadFollBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PADFOLLBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "DiffBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_DIFFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "OnBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ONBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "OffBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_OFFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "PixInvBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PIXINVBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "PrBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PRBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "PrSFBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PRSFBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "RefrBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_REFRBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "ReadoutBufBp") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_READOUTBUFBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "ApsROSFBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSROSFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "AdcCompBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCCOMPBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "ColSelLowBn") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_COLSELLOWBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "DACBufBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_DACBUFBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "LcolTimeoutBn") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_LCOLTIMEOUTBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "AEPdBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPDBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "AEPuXBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPUXBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "AEPuYBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPUYBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "IFRefrBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_IFREFRBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "IFThrBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_IFTHRBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "BiasBuffer") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_BIASBUFFER, generateCoarseFineBias(node));
				}
				else if (nodeName == "SSP") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_SSP, generateShiftedSourceBias(node));
				}
				else if (nodeName == "SSN") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_SSN, generateShiftedSourceBias(node));
				}
			}

			if (IS_DAVIS640H(devInfo.chipID)) {
				if (nodeName == "ApsCas") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_APSCAS, generateVDACBias(node));
				}
				else if (nodeName == "OVG1Lo") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_OVG1LO, generateVDACBias(node));
				}
				else if (nodeName == "OVG2Lo") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_OVG2LO, generateVDACBias(node));
				}
				else if (nodeName == "TX2OVG2Hi") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_TX2OVG2HI, generateVDACBias(node));
				}
				else if (nodeName == "Gnd07") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_GND07, generateVDACBias(node));
				}
				else if (nodeName == "AdcTestVoltage") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCTESTVOLTAGE, generateVDACBias(node));
				}
				else if (nodeName == "AdcRefHigh") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCREFHIGH, generateVDACBias(node));
				}
				else if (nodeName == "AdcRefLow") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCREFLOW, generateVDACBias(node));
				}
				else if (nodeName == "IFRefrBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_IFREFRBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "IFThrBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_IFTHRBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "LocalBufBn") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_LOCALBUFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "PadFollBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PADFOLLBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "PixInvBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PIXINVBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "DiffBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_DIFFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "OnBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ONBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "OffBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_OFFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "PrBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PRBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "PrSFBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PRSFBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "RefrBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_REFRBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "ArrayBiasBufferBn") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ARRAYBIASBUFFERBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "ArrayLogicBufferBn") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ARRAYLOGICBUFFERBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "FalltimeBn") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_FALLTIMEBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "RisetimeBp") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_RISETIMEBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "ReadoutBufBp") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_READOUTBUFBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "ApsROSFBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_APSROSFBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "AdcCompBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCCOMPBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "DACBufBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_DACBUFBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "LcolTimeoutBn") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_LCOLTIMEOUTBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "AEPdBn") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_AEPDBN, generateCoarseFineBias(node));
				}
				else if (nodeName == "AEPuXBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_AEPUXBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "AEPuYBp") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_AEPUYBP, generateCoarseFineBias(node));
				}
				else if (nodeName == "BiasBuffer") {
					device->configSet(
						DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_BIASBUFFER, generateCoarseFineBias(node));
				}
				else if (nodeName == "SSP") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_SSP, generateShiftedSourceBias(node));
				}
				else if (nodeName == "SSN") {
					device->configSet(DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_SSN, generateShiftedSourceBias(node));
				}
			}
		}
	}

	void chipConfigCreateDynamic(const struct caer_davis_info *devInfo) {
		auto chipPath = chipIDToName(devInfo->chipID, true) + "chip/";

		// Chip configuration shift register.
		config.add(chipPath + "DigitalMux0", dv::ConfigOption::intOption("Digital debug multiplexer 0.", 0, 0, 15));
		config.add(chipPath + "DigitalMux1", dv::ConfigOption::intOption("Digital debug multiplexer 1.", 0, 0, 15));
		config.add(chipPath + "DigitalMux2", dv::ConfigOption::intOption("Digital debug multiplexer 2.", 0, 0, 15));
		config.add(chipPath + "DigitalMux3", dv::ConfigOption::intOption("Digital debug multiplexer 3.", 0, 0, 15));
		config.add(chipPath + "AnalogMux0", dv::ConfigOption::intOption("Analog debug multiplexer 0.", 0, 0, 15));
		config.add(chipPath + "AnalogMux1", dv::ConfigOption::intOption("Analog debug multiplexer 1.", 0, 0, 15));
		config.add(chipPath + "AnalogMux2", dv::ConfigOption::intOption("Analog debug multiplexer 2.", 0, 0, 15));
		config.add(chipPath + "BiasMux0", dv::ConfigOption::intOption("Bias debug multiplexer 0.", 0, 0, 15));

		config.add(chipPath + "ResetCalibNeuron",
			dv::ConfigOption::boolOption("Turn off the integrate and fire calibration neuron (bias generator).", true));
		config.add(chipPath + "TypeNCalibNeuron",
			dv::ConfigOption::boolOption("Make the integrate and fire calibration neuron measure N-type biases; "
										 "otherwise measures P-type biases.",
				false));
		config.add(chipPath + "ResetTestPixel",
			dv::ConfigOption::boolOption("Keep the test pixel in reset (disabled).", true));
		config.add(chipPath + "AERnArow", dv::ConfigOption::boolOption("Use nArow in the AER state machine.", false));
		config.add(chipPath + "UseAOut",
			dv::ConfigOption::boolOption("Enable analog pads for the analog debug multiplexers outputs.", false));

		// No GlobalShutter flag here, it's controlled by the APS module's GS flag, and libcaer
		// ensures that both the chip SR and the APS module flags are kept in sync.

		if (IS_DAVIS240A(devInfo->chipID)) {
			config.add(chipPath + "SpecialPixelControl",
				dv::ConfigOption::boolOption("Enable experimental hot-pixels suppression circuit.", false));
		}
		if (IS_DAVIS240B(devInfo->chipID)) {
			config.add(chipPath + "SpecialPixelControl",
				dv::ConfigOption::boolOption("Enable experimental pixel stripes on right side of array.", false));
		}

		if (IS_DAVIS128(devInfo->chipID) || IS_DAVIS208(devInfo->chipID) || IS_DAVIS346(devInfo->chipID)
			|| IS_DAVIS640(devInfo->chipID) || IS_DAVIS640H(devInfo->chipID)) {
			config.add(chipPath + "SelectGrayCounter",
				dv::ConfigOption::boolOption(
					"Select which gray counter to use with the internal ADC: '0' means the external gray counter is "
					"used, which has to be supplied off-chip. '1' means the on-chip gray counter is used instead.",
					true));
		}

		if (IS_DAVIS346(devInfo->chipID) || IS_DAVIS640(devInfo->chipID) || IS_DAVIS640H(devInfo->chipID)) {
			config.add(chipPath + "TestADC",
				dv::ConfigOption::boolOption(
					"Test ADC functionality: if true, the ADC takes its input voltage not from the pixel, but from the "
					"VDAC 'AdcTestVoltage'. If false, the voltage comes from the pixels.",
					false));
		}

		if (IS_DAVIS208(devInfo->chipID)) {
			config.add(chipPath + "SelectPreAmpAvg",
				dv::ConfigOption::boolOption(
					"If 1, connect PreAmpAvgxA to calibration neuron, if 0, commongate.", false));
			config.add(chipPath + "SelectBiasRefSS",
				dv::ConfigOption::boolOption("If 1, select Nbias Blk1N, if 0, VDAC VblkV2.", false));
			config.add(chipPath + "SelectSense", dv::ConfigOption::boolOption("Enable Sensitive pixels.", true));
			config.add(chipPath + "SelectPosFb", dv::ConfigOption::boolOption("Enable PosFb pixels.", false));
			config.add(chipPath + "SelectHighPass", dv::ConfigOption::boolOption("Enable HighPass pixels.", false));
		}

		if (IS_DAVIS640H(devInfo->chipID)) {
			config.add(chipPath + "AdjustOVG1Lo", dv::ConfigOption::boolOption("Adjust OVG1 Low.", true));
			config.add(chipPath + "AdjustOVG2Lo", dv::ConfigOption::boolOption("Adjust OVG2 Low.", false));
			config.add(chipPath + "AdjustTX2OVG2Hi", dv::ConfigOption::boolOption("Adjust TX2OVG2Hi.", false));
		}

		config.add(chipPath + "BiasEnable", dv::ConfigOption::boolOption("Enable bias generator to power chip.", true));

		config.setPriorityOptions({chipPath});
	}

	void chipConfigSend(const struct caer_davis_info *devInfo) {
		auto chipPath = chipIDToName(devInfo->chipID, true) + "chip/";

		// All chips have the same parameter address for the same setting!
		device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX0,
			static_cast<uint32_t>(config.getInt(chipPath + "DigitalMux0")));
		device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX1,
			static_cast<uint32_t>(config.getInt(chipPath + "DigitalMux1")));
		device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX2,
			static_cast<uint32_t>(config.getInt(chipPath + "DigitalMux2")));
		device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX3,
			static_cast<uint32_t>(config.getInt(chipPath + "DigitalMux3")));
		device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX0,
			static_cast<uint32_t>(config.getInt(chipPath + "AnalogMux0")));
		device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX1,
			static_cast<uint32_t>(config.getInt(chipPath + "AnalogMux1")));
		device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX2,
			static_cast<uint32_t>(config.getInt(chipPath + "AnalogMux2")));
		device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_BIASMUX0,
			static_cast<uint32_t>(config.getInt(chipPath + "BiasMux0")));

		device.configSet(
			DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_RESETCALIBNEURON, config.getBool(chipPath + "ResetCalibNeuron"));
		device.configSet(
			DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_TYPENCALIBNEURON, config.getBool(chipPath + "TypeNCalibNeuron"));
		device.configSet(
			DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_RESETTESTPIXEL, config.getBool(chipPath + "ResetTestPixel"));
		device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_AERNAROW, config.getBool(chipPath + "AERnArow"));
		device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_USEAOUT, config.getBool(chipPath + "UseAOut"));

		if (IS_DAVIS240A(devInfo->chipID) || IS_DAVIS240B(devInfo->chipID)) {
			device.configSet(DAVIS_CONFIG_CHIP, DAVIS240_CONFIG_CHIP_SPECIALPIXELCONTROL,
				config.getBool(chipPath + "SpecialPixelControl"));
		}

		if (IS_DAVIS128(devInfo->chipID) || IS_DAVIS208(devInfo->chipID) || IS_DAVIS346(devInfo->chipID)
			|| IS_DAVIS640(devInfo->chipID) || IS_DAVIS640H(devInfo->chipID)) {
			device.configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER,
				config.getBool(chipPath + "SelectGrayCounter"));
		}

		if (IS_DAVIS346(devInfo->chipID) || IS_DAVIS640(devInfo->chipID) || IS_DAVIS640H(devInfo->chipID)) {
			device.configSet(DAVIS_CONFIG_CHIP, DAVIS346_CONFIG_CHIP_TESTADC, config.getBool(chipPath + "TestADC"));
		}

		if (IS_DAVIS208(devInfo->chipID)) {
			device.configSet(
				DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTPREAMPAVG, config.getBool(chipPath + "SelectPreAmpAvg"));
			device.configSet(
				DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTBIASREFSS, config.getBool(chipPath + "SelectBiasRefSS"));
			device.configSet(
				DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTSENSE, config.getBool(chipPath + "SelectSense"));
			device.configSet(
				DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTPOSFB, config.getBool(chipPath + "SelectPosFb"));
			device.configSet(
				DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTHIGHPASS, config.getBool(chipPath + "SelectHighPass"));
		}

		if (IS_DAVIS640H(devInfo->chipID)) {
			device.configSet(
				DAVIS_CONFIG_CHIP, DAVIS640H_CONFIG_CHIP_ADJUSTOVG1LO, config.getBool(chipPath + "AdjustOVG1Lo"));
			device.configSet(
				DAVIS_CONFIG_CHIP, DAVIS640H_CONFIG_CHIP_ADJUSTOVG2LO, config.getBool(chipPath + "AdjustOVG2Lo"));
			device.configSet(
				DAVIS_CONFIG_CHIP, DAVIS640H_CONFIG_CHIP_ADJUSTTX2OVG2HI, config.getBool(chipPath + "AdjustTX2OVG2Hi"));
		}

		device.configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN_CHIP, config.getBool(chipPath + "BiasEnable"));
	}

	static void chipConfigListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		UNUSED_ARGUMENT(node);

		auto device        = static_cast<libcaer::devices::davis *>(userData);
		const auto devInfo = device->infoGet();

		std::string key{changeKey};

		if (event == DVCFG_ATTRIBUTE_MODIFIED) {
			if (changeType == DVCFG_TYPE_INT && key == "DigitalMux0") {
				device->configSet(
					DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX0, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "DigitalMux1") {
				device->configSet(
					DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX1, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "DigitalMux2") {
				device->configSet(
					DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX2, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "DigitalMux3") {
				device->configSet(
					DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX3, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "AnalogMux0") {
				device->configSet(
					DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX0, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "AnalogMux1") {
				device->configSet(
					DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX1, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "AnalogMux2") {
				device->configSet(
					DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX2, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "BiasMux0") {
				device->configSet(
					DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_BIASMUX0, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "ResetCalibNeuron") {
				device->configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_RESETCALIBNEURON, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "TypeNCalibNeuron") {
				device->configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_TYPENCALIBNEURON, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "ResetTestPixel") {
				device->configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_RESETTESTPIXEL, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "AERnArow") {
				device->configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_AERNAROW, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "UseAOut") {
				device->configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_USEAOUT, changeValue.boolean);
			}
			else if ((IS_DAVIS240A(devInfo.chipID) || IS_DAVIS240B(devInfo.chipID)) && changeType == DVCFG_TYPE_BOOL
					 && key == "SpecialPixelControl") {
				device->configSet(DAVIS_CONFIG_CHIP, DAVIS240_CONFIG_CHIP_SPECIALPIXELCONTROL, changeValue.boolean);
			}
			else if ((IS_DAVIS128(devInfo.chipID) || IS_DAVIS208(devInfo.chipID) || IS_DAVIS346(devInfo.chipID)
						 || IS_DAVIS640(devInfo.chipID) || IS_DAVIS640H(devInfo.chipID))
					 && changeType == DVCFG_TYPE_BOOL && key == "SelectGrayCounter") {
				device->configSet(DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER, changeValue.boolean);
			}
			else if ((IS_DAVIS346(devInfo.chipID) || IS_DAVIS640(devInfo.chipID) || IS_DAVIS640H(devInfo.chipID))
					 && changeType == DVCFG_TYPE_BOOL && key == "TestADC") {
				device->configSet(DAVIS_CONFIG_CHIP, DAVIS346_CONFIG_CHIP_TESTADC, changeValue.boolean);
			}

			if (IS_DAVIS208(devInfo.chipID)) {
				if (changeType == DVCFG_TYPE_BOOL && key == "SelectPreAmpAvg") {
					device->configSet(DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTPREAMPAVG, changeValue.boolean);
				}
				else if (changeType == DVCFG_TYPE_BOOL && key == "SelectBiasRefSS") {
					device->configSet(DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTBIASREFSS, changeValue.boolean);
				}
				else if (changeType == DVCFG_TYPE_BOOL && key == "SelectSense") {
					device->configSet(DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTSENSE, changeValue.boolean);
				}
				else if (changeType == DVCFG_TYPE_BOOL && key == "SelectPosFb") {
					device->configSet(DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTPOSFB, changeValue.boolean);
				}
				else if (changeType == DVCFG_TYPE_BOOL && key == "SelectHighPass") {
					device->configSet(DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTHIGHPASS, changeValue.boolean);
				}
			}

			if (IS_DAVIS640H(devInfo.chipID)) {
				if (changeType == DVCFG_TYPE_BOOL && key == "AdjustOVG1Lo") {
					device->configSet(DAVIS_CONFIG_CHIP, DAVIS640H_CONFIG_CHIP_ADJUSTOVG1LO, changeValue.boolean);
				}
				else if (changeType == DVCFG_TYPE_BOOL && key == "AdjustOVG2Lo") {
					device->configSet(DAVIS_CONFIG_CHIP, DAVIS640H_CONFIG_CHIP_ADJUSTOVG2LO, changeValue.boolean);
				}
				else if (changeType == DVCFG_TYPE_BOOL && key == "AdjustTX2OVG2Hi") {
					device->configSet(DAVIS_CONFIG_CHIP, DAVIS640H_CONFIG_CHIP_ADJUSTTX2OVG2HI, changeValue.boolean);
				}
			}

			if (changeType == DVCFG_TYPE_BOOL && key == "BiasEnable") {
				device->configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN_CHIP, changeValue.boolean);
			}
		}
	}

	static void multiplexerConfigCreate(dv::RuntimeConfig &config) {
		// Subsystem 0: Multiplexer
		config.add("multiplexer/Run", dv::ConfigOption::boolOption("Enable multiplexer state machine.", true));
		config.add("multiplexer/TimestampRun", dv::ConfigOption::boolOption("Enable µs-timestamp generation.", true));
		config.add("multiplexer/TimestampReset",
			dv::ConfigOption::buttonOption("Reset timestamps to zero.", "Reset timestamps"));
		config.add("multiplexer/DropDVSOnTransferStall",
			dv::ConfigOption::boolOption("Drop Polarity events when USB FIFO is full.", true));
		config.add("multiplexer/DropExtInputOnTransferStall",
			dv::ConfigOption::boolOption("Drop ExternalInput events when USB FIFO is full.", true));

		config.setPriorityOptions({"multiplexer/"});
	}

	void multiplexerConfigCreateDynamic(const struct caer_davis_info *devInfo) {
		// Device event statistics.
		if (devInfo->muxHasStatistics) {
			config.add("statistics/muxDroppedDVS",
				dv::ConfigOption::statisticOption("Number of dropped DVS events due to USB full."));
			config.add("statistics/muxDroppedExtInput",
				dv::ConfigOption::statisticOption("Number of dropped External Input events due to USB full."));

			auto statNode = moduleNode.getRelativeNode("statistics/");

			statNode.attributeUpdaterAdd("muxDroppedDVS", dv::CfgType::LONG, &statisticsUpdater, &device);
			statNode.attributeUpdaterAdd("muxDroppedExtInput", dv::CfgType::LONG, &statisticsUpdater, &device);

			config.setPriorityOptions({"statistics/"});
		}
	}

	void multiplexerConfigSend() {
		device.configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, false);
		device.configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL,
			config.getBool("multiplexer/DropDVSOnTransferStall"));
		device.configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL,
			config.getBool("multiplexer/DropExtInputOnTransferStall"));
		device.configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN, config.getBool("multiplexer/TimestampRun"));
		device.configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, config.getBool("multiplexer/Run"));
	}

	static void multiplexerConfigListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		UNUSED_ARGUMENT(node);

		auto device = static_cast<libcaer::devices::davis *>(userData);

		std::string key{changeKey};

		if (event == DVCFG_ATTRIBUTE_MODIFIED) {
			if (changeType == DVCFG_TYPE_BOOL && key == "TimestampReset" && changeValue.boolean) {
				device->configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, changeValue.boolean);

				dvConfigNodeAttributeBooleanReset(node, changeKey);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "DropDVSOnTransferStall") {
				device->configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "DropExtInputOnTransferStall") {
				device->configSet(
					DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "TimestampRun") {
				device->configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "Run") {
				device->configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, changeValue.boolean);
			}
		}
	}

	static void dvsConfigCreate(dv::RuntimeConfig &config) {
		// Subsystem 1: DVS
		config.add("dvs/WaitOnTransferStall",
			dv::ConfigOption::boolOption(
				"On event FIFO full, wait to ACK until again empty if true, or just continue ACKing if false.", false));
		config.add("dvs/ExternalAERControl",
			dv::ConfigOption::boolOption(
				"Don't drive AER ACK pin from FPGA (also switch to 'Frames only' mode).", false));

		config.setPriorityOptions({"dvs/"});
	}

	void dvsConfigCreateDynamic(const struct caer_davis_info *devInfo) {
		if (devInfo->dvsHasPixelFilter) {
			config.add(
				"dvs/PixelFilter/Pixel0Row", dv::ConfigOption::intOption("Row/Y address of pixel 0 to filter out.",
												 devInfo->dvsSizeY, 0, devInfo->dvsSizeY));
			config.add("dvs/PixelFilter/Pixel0Column",
				dv::ConfigOption::intOption(
					"Column/X address of pixel 0 to filter out.", devInfo->dvsSizeX, 0, devInfo->dvsSizeX));
			config.add(
				"dvs/PixelFilter/Pixel1Row", dv::ConfigOption::intOption("Row/Y address of pixel 1 to filter out.",
												 devInfo->dvsSizeY, 0, devInfo->dvsSizeY));
			config.add("dvs/PixelFilter/Pixel1Column",
				dv::ConfigOption::intOption(
					"Column/X address of pixel 1 to filter out.", devInfo->dvsSizeX, 0, devInfo->dvsSizeX));
			config.add(
				"dvs/PixelFilter/Pixel2Row", dv::ConfigOption::intOption("Row/Y address of pixel 2 to filter out.",
												 devInfo->dvsSizeY, 0, devInfo->dvsSizeY));
			config.add("dvs/PixelFilter/Pixel2Column",
				dv::ConfigOption::intOption(
					"Column/X address of pixel 2 to filter out.", devInfo->dvsSizeX, 0, devInfo->dvsSizeX));
			config.add(
				"dvs/PixelFilter/Pixel3Row", dv::ConfigOption::intOption("Row/Y address of pixel 3 to filter out.",
												 devInfo->dvsSizeY, 0, devInfo->dvsSizeY));
			config.add("dvs/PixelFilter/Pixel3Column",
				dv::ConfigOption::intOption(
					"Column/X address of pixel 3 to filter out.", devInfo->dvsSizeX, 0, devInfo->dvsSizeX));
			config.add(
				"dvs/PixelFilter/Pixel4Row", dv::ConfigOption::intOption("Row/Y address of pixel 4 to filter out.",
												 devInfo->dvsSizeY, 0, devInfo->dvsSizeY));
			config.add("dvs/PixelFilter/Pixel4Column",
				dv::ConfigOption::intOption(
					"Column/X address of pixel 4 to filter out.", devInfo->dvsSizeX, 0, devInfo->dvsSizeX));
			config.add(
				"dvs/PixelFilter/Pixel5Row", dv::ConfigOption::intOption("Row/Y address of pixel 5 to filter out.",
												 devInfo->dvsSizeY, 0, devInfo->dvsSizeY));
			config.add("dvs/PixelFilter/Pixel5Column",
				dv::ConfigOption::intOption(
					"Column/X address of pixel 5 to filter out.", devInfo->dvsSizeX, 0, devInfo->dvsSizeX));
			config.add(
				"dvs/PixelFilter/Pixel6Row", dv::ConfigOption::intOption("Row/Y address of pixel 6 to filter out.",
												 devInfo->dvsSizeY, 0, devInfo->dvsSizeY));
			config.add("dvs/PixelFilter/Pixel6Column",
				dv::ConfigOption::intOption(
					"Column/X address of pixel 6 to filter out.", devInfo->dvsSizeX, 0, devInfo->dvsSizeX));
			config.add(
				"dvs/PixelFilter/Pixel7Row", dv::ConfigOption::intOption("Row/Y address of pixel 7 to filter out.",
												 devInfo->dvsSizeY, 0, devInfo->dvsSizeY));
			config.add("dvs/PixelFilter/Pixel7Column",
				dv::ConfigOption::intOption(
					"Column/X address of pixel 7 to filter out.", devInfo->dvsSizeX, 0, devInfo->dvsSizeX));
			config.add("dvs/PixelFilter/AutoTrain",
				dv::ConfigOption::buttonOption(
					"Set hardware pixel filter up automatically using software hot-pixel detection.",
					"Train hot-pixel filter"));

			config.setPriorityOptions({"dvs/PixelFilter/"});
		}

		if (devInfo->dvsHasBackgroundActivityFilter) {
			config.add("dvs/NoiseFilter/Enable",
				dv::ConfigOption::boolOption("Filter noise using hardware filter on FPGA.", true));
			config.add("dvs/NoiseFilter/Time",
				dv::ConfigOption::intOption("Maximum time difference for events to be considered correlated and not be "
											"filtered out (in 250µs units).",
					8, 0, (0x01 << 12) - 1));
			config.add("dvs/RateFilter/Enable",
				dv::ConfigOption::boolOption("Limit pixel firing rate using hardware filter on FPGA.", false));
			config.add("dvs/RateFilter/Time",
				dv::ConfigOption::intOption(
					"Minimum time between events to not be filtered out (in 250µs units).", 1, 0, (0x01 << 12) - 1));

			config.setPriorityOptions({"dvs/NoiseFilter/Enable", "dvs/RateFilter/Enable"});
		}

		if (devInfo->dvsHasROIFilter) {
			config.add(
				"dvs/ROIFilter/StartColumn", dv::ConfigOption::intOption("Column/X address of ROI filter start point.",
												 0, 0, static_cast<int16_t>(devInfo->dvsSizeX - 1)));
			config.add("dvs/ROIFilter/StartRow", dv::ConfigOption::intOption("Row/Y address of ROI filter start point.",
													 0, 0, static_cast<int16_t>(devInfo->dvsSizeY - 1)));
			config.add("dvs/ROIFilter/EndColumn",
				dv::ConfigOption::intOption("Column/X address of ROI filter end point.",
					static_cast<int16_t>(devInfo->dvsSizeX - 1), 0, static_cast<int16_t>(devInfo->dvsSizeX - 1)));
			config.add("dvs/ROIFilter/EndRow",
				dv::ConfigOption::intOption("Row/Y address of ROI filter end point.",
					static_cast<int16_t>(devInfo->dvsSizeY - 1), 0, static_cast<int16_t>(devInfo->dvsSizeY - 1)));

			config.setPriorityOptions({"dvs/ROIFilter/"});
		}

		if (devInfo->dvsHasSkipFilter) {
			config.add("dvs/SkipFilter/Enable", dv::ConfigOption::boolOption("Skip one event every N.", false));
			config.add("dvs/SkipFilter/SkipEveryEvents",
				dv::ConfigOption::intOption(
					"Number of events to let through before skipping one.", 1, 1, (0x01 << 8) - 1));

			config.setPriorityOptions({"dvs/SkipFilter/"});
		}

		if (devInfo->dvsHasPolarityFilter) {
			config.add("dvs/PolarityFilter/Flatten",
				dv::ConfigOption::boolOption("Change all event polarities to OFF.", false));
			config.add("dvs/PolarityFilter/Suppress",
				dv::ConfigOption::boolOption("Suppress events of a certain polarity.", false));
			config.add("dvs/PolarityFilter/SuppressType",
				dv::ConfigOption::boolOption("Polarity to suppress (false=OFF, true=ON).", false));

			config.setPriorityOptions({"dvs/PolarityFilter/"});
		}

		if (devInfo->dvsHasStatistics) {
			config.add("statistics/dvsEventsRow", dv::ConfigOption::statisticOption("Number of row events handled."));
			config.add(
				"statistics/dvsEventsColumn", dv::ConfigOption::statisticOption("Number of column events handled."));
			config.add("statistics/dvsEventsDropped",
				dv::ConfigOption::statisticOption("Number of dropped events (groups of events)."));

			auto statNode = moduleNode.getRelativeNode("statistics/");

			statNode.attributeUpdaterAdd("dvsEventsRow", dv::CfgType::LONG, &statisticsUpdater, &device);
			statNode.attributeUpdaterAdd("dvsEventsColumn", dv::CfgType::LONG, &statisticsUpdater, &device);
			statNode.attributeUpdaterAdd("dvsEventsDropped", dv::CfgType::LONG, &statisticsUpdater, &device);

			config.setPriorityOptions({"statistics/"});

			if (devInfo->dvsHasPixelFilter) {
				config.add("statistics/dvsFilteredPixel",
					dv::ConfigOption::statisticOption("Number of events filtered out by the Pixel Filter."));

				statNode.attributeUpdaterAdd("dvsFilteredPixel", dv::CfgType::LONG, &statisticsUpdater, &device);
			}

			if (devInfo->dvsHasBackgroundActivityFilter) {
				config.add("statistics/dvsFilteredNoise",
					dv::ConfigOption::statisticOption("Number of events filtered out by the Noise Filter."));
				config.add("statistics/dvsFilteredRate",
					dv::ConfigOption::statisticOption("Number of events filtered out by the Rate Filter."));

				statNode.attributeUpdaterAdd("dvsFilteredNoise", dv::CfgType::LONG, &statisticsUpdater, &device);
				statNode.attributeUpdaterAdd("dvsFilteredRate", dv::CfgType::LONG, &statisticsUpdater, &device);
			}
		}
	}

	void dvsConfigSend(const struct caer_davis_info *devInfo) {
		device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL,
			static_cast<uint32_t>(config.getBool("dvs/WaitOnTransferStall")));
		device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL,
			static_cast<uint32_t>(config.getBool("dvs/ExternalAERControl")));

		if (devInfo->dvsHasPixelFilter) {
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel0Row")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel0Column")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel1Row")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel1Column")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel2Row")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel2Column")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel3Row")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel3Column")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel4Row")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel4Column")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel5Row")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel5Column")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel6Row")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel6Column")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel7Row")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN,
				static_cast<uint32_t>(config.getInt("dvs/PixelFilter/Pixel7Column")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_AUTO_TRAIN,
				config.getBool("dvs/PixelFilter/AutoTrain"));
		}

		if (devInfo->dvsHasBackgroundActivityFilter) {
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY,
				config.getBool("dvs/NoiseFilter/Enable"));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME,
				static_cast<uint32_t>(config.getInt("dvs/NoiseFilter/Time")));
			device.configSet(
				DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD, config.getBool("dvs/RateFilter/Enable"));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME,
				static_cast<uint32_t>(config.getInt("dvs/RateFilter/Time")));
		}

		if (devInfo->dvsHasROIFilter) {
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN,
				static_cast<uint32_t>(config.getInt("dvs/ROIFilter/StartColumn")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW,
				static_cast<uint32_t>(config.getInt("dvs/ROIFilter/StartRow")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN,
				static_cast<uint32_t>(config.getInt("dvs/ROIFilter/EndColumn")));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW,
				static_cast<uint32_t>(config.getInt("dvs/ROIFilter/EndRow")));
		}

		if (devInfo->dvsHasSkipFilter) {
			device.configSet(
				DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS, config.getBool("dvs/SkipFilter/Enable"));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY,
				static_cast<uint32_t>(config.getInt("dvs/SkipFilter/SkipEveryEvents")));
		}

		if (devInfo->dvsHasPolarityFilter) {
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN,
				config.getBool("dvs/PolarityFilter/Flatten"));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS,
				config.getBool("dvs/PolarityFilter/Suppress"));
			device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE,
				config.getBool("dvs/PolarityFilter/SuppressType"));
		}

		bool runDVS = (config.getString("dataMode").find("Events") != std::string::npos);
		device.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, runDVS);
	}

	static void dvsConfigListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		auto device = static_cast<libcaer::devices::davis *>(userData);

		std::string key{dv::Cfg::Node(node).getName() + "/" + changeKey};

		if (event == DVCFG_ATTRIBUTE_MODIFIED) {
			if (changeType == DVCFG_TYPE_BOOL && key == "dvs/WaitOnTransferStall") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "dvs/ExternalAERControl") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel0Row") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel0Column") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel1Row") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel1Column") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel2Row") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel2Column") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel3Row") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel3Column") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel4Row") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel4Column") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel5Row") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel5Column") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel6Row") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel6Column") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel7Row") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PixelFilter/Pixel7Column") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "PixelFilter/AutoTrain" && changeValue.boolean) {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_AUTO_TRAIN, changeValue.boolean);

				dvConfigNodeAttributeBooleanReset(node, changeKey);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "NoiseFilter/Enable") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_INT && key == "NoiseFilter/Time") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME,
					static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "RateFilter/Enable") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_INT && key == "RateFilter/Time") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME,
					static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "ROIFilter/StartColumn") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN,
					static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "ROIFilter/StartRow") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "ROIFilter/EndColumn") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "ROIFilter/EndRow") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "SkipFilter/Enable") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_INT && key == "SkipFilter/SkipEveryEvents") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY,
					static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "PolarityFilter/Flatten") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "PolarityFilter/Suppress") {
				device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "PolarityFilter/SuppressType") {
				device->configSet(
					DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE, changeValue.boolean);
			}
		}
	}

	static void apsConfigCreate(dv::RuntimeConfig &config) {
		// Subsystem 2: APS ADC
		config.add("aps/WaitOnTransferStall",
			dv::ConfigOption::boolOption(
				"On event FIFO full, pause and wait for free space. This ensures no APS pixels are dropped.", true));

		config.add(
			"aps/Exposure", dv::ConfigOption::intOption("Set exposure time (in µs).", 4000, 0, (0x01 << 22) - 1));
		config.add("aps/FrameInterval",
			dv::ConfigOption::intOption("Set time between frames (in µs).", 40000, 0, (0x01 << 23) - 1));

		config.add("aps/TakeSnapShot", dv::ConfigOption::buttonOption("Take a single frame capture.", "Snap!"));
		config.add("aps/AutoExposure",
			dv::ConfigOption::boolOption(
				"Enable automatic exposure control, to react to changes in lighting conditions.", true));

		config.add("aps/FrameMode",
			dv::ConfigOption::listOption("Select frame output mode.", "Default", {"Default", "Grayscale", "Original"}));

		config.setPriorityOptions({"aps/FrameMode", "aps/AutoExposure", "aps/Exposure", "aps/FrameInterval"});
	}

	void apsConfigCreateDynamic(const struct caer_davis_info *devInfo) {
		config.add("aps/StartColumn", dv::ConfigOption::intOption("Column/X address of ROI start point.", 0, 0,
										  static_cast<int16_t>(devInfo->apsSizeX - 1)));
		config.add("aps/StartRow", dv::ConfigOption::intOption("Row/Y address of ROI start point.", 0, 0,
									   static_cast<int16_t>(devInfo->apsSizeY - 1)));
		config.add("aps/EndColumn",
			dv::ConfigOption::intOption("Column/X address of ROI end point.",
				static_cast<int16_t>(devInfo->apsSizeX - 1), 0, static_cast<int16_t>(devInfo->apsSizeX - 1)));
		config.add("aps/EndRow",
			dv::ConfigOption::intOption("Row/Y address of ROI end point.", static_cast<int16_t>(devInfo->apsSizeY - 1),
				0, static_cast<int16_t>(devInfo->apsSizeY - 1)));

		if (devInfo->apsHasGlobalShutter) {
			// Only support GS on chips that have it available.
			config.add("aps/GlobalShutter",
				dv::ConfigOption::boolOption("Enable global-shutter versus rolling-shutter mode.", true));
		}

		// DAVIS RGB has additional timing counters.
		if (IS_DAVIS640H(devInfo->chipID)) {
			config.add(
				"aps/TransferTime", dv::ConfigOption::intOption(
										"Transfer time counter (2 in GS, 1 in RS, in cycles).", 1500, 0, (60 * 2048)));
			config.add(
				"aps/RSFDSettleTime", dv::ConfigOption::intOption("RS counter 0 (in cycles).", 1000, 0, (60 * 128)));
			config.add(
				"aps/GSPDResetTime", dv::ConfigOption::intOption("GS counter 0 (in cycles).", 1000, 0, (60 * 128)));
			config.add(
				"aps/GSResetFallTime", dv::ConfigOption::intOption("GS counter 1 (in cycles).", 1000, 0, (60 * 128)));
			config.add(
				"aps/GSTXFallTime", dv::ConfigOption::intOption("GS counter 3 (in cycles).", 1000, 0, (60 * 128)));
			config.add(
				"aps/GSFDResetTime", dv::ConfigOption::intOption("GS counter 4 (in cycles).", 1000, 0, (60 * 128)));
		}
	}

	void apsConfigSend(const struct caer_davis_info *devInfo) {
		device.configSet(
			DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL, config.getBool("aps/WaitOnTransferStall"));

		if (devInfo->apsHasGlobalShutter) {
			device.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, config.getBool("aps/GlobalShutter"));
		}

		device.configSet(
			DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, static_cast<uint32_t>(config.getInt("aps/StartColumn")));
		device.configSet(
			DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, static_cast<uint32_t>(config.getInt("aps/StartRow")));
		device.configSet(
			DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, static_cast<uint32_t>(config.getInt("aps/EndColumn")));
		device.configSet(
			DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, static_cast<uint32_t>(config.getInt("aps/EndRow")));

		// Initialize exposure in backend (libcaer), so that value is synchronized with it.
		device.configSet(
			DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, static_cast<uint32_t>(config.getInt("aps/Exposure")));

		moduleNode.getRelativeNode("aps/").attributeUpdaterAdd(
			"Exposure", dv::CfgType::INT, &apsExposureUpdater, &device);

		device.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_INTERVAL,
			static_cast<uint32_t>(config.getInt("aps/FrameInterval")));

		// DAVIS RGB extra timing support.
		if (IS_DAVIS640H(devInfo->chipID)) {
			device.configSet(DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_TRANSFER,
				static_cast<uint32_t>(config.getInt("aps/TransferTime")));
			device.configSet(DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_RSFDSETTLE,
				static_cast<uint32_t>(config.getInt("aps/RSFDSettleTime")));
			device.configSet(DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSPDRESET,
				static_cast<uint32_t>(config.getInt("aps/GSPDResetTime")));
			device.configSet(DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSRESETFALL,
				static_cast<uint32_t>(config.getInt("aps/GSResetFallTime")));
			device.configSet(DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSTXFALL,
				static_cast<uint32_t>(config.getInt("aps/GSTXFallTime")));
			device.configSet(DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSFDRESET,
				static_cast<uint32_t>(config.getInt("aps/GSFDResetTime")));
		}

		device.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, config.getBool("aps/AutoExposure"));

		device.configSet(
			DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_MODE, mapFrameMode(config.getString("aps/FrameMode")));

		bool runAPS = (config.getString("dataMode").find("Frames") != std::string::npos);
		device.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, runAPS);
	}

	static void apsConfigListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		UNUSED_ARGUMENT(node);

		auto device = static_cast<libcaer::devices::davis *>(userData);

		std::string key{changeKey};

		if (event == DVCFG_ATTRIBUTE_MODIFIED) {
			if (changeType == DVCFG_TYPE_BOOL && key == "WaitOnTransferStall") {
				device->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "GlobalShutter") {
				device->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_INT && key == "StartColumn") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "StartRow") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "EndColumn") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "EndRow") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "Exposure") {
				// Exposure is forbidden to be set if AutoExposure is enabled!
				if (!device->configGet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE)) {
					device->configSet(
						DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, static_cast<uint32_t>(changeValue.iint));
				}
			}
			else if (changeType == DVCFG_TYPE_INT && key == "FrameInterval") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_INTERVAL, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "TransferTime") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_TRANSFER, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "RSFDSettleTime") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_RSFDSETTLE, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "GSPDResetTime") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSPDRESET, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "GSResetFallTime") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSRESETFALL, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "GSTXFallTime") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSTXFALL, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "GSFDResetTime") {
				device->configSet(
					DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSFDRESET, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "TakeSnapShot" && changeValue.boolean) {
				device->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SNAPSHOT, changeValue.boolean);

				dvConfigNodeAttributeBooleanReset(node, changeKey);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "AutoExposure") {
				device->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_STRING && key == "FrameMode") {
				device->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_MODE, mapFrameMode(changeValue.string));
			}
		}
	}

	static void imuConfigCreate(dv::RuntimeConfig &config) {
		// Subsystem 3: IMU
		config.add("imu/RunAccelerometer", dv::ConfigOption::boolOption("Enable accelerometer.", true));
		config.add("imu/RunGyroscope", dv::ConfigOption::boolOption("Enable gyroscope.", true));
		config.add("imu/RunTemperature", dv::ConfigOption::boolOption("Enable temperature sensor.", true));

		config.add("imu/SampleRateDivider", dv::ConfigOption::intOption("Sample-rate divider value.", 0, 0, 255));

		config.add("imu/AccelFullScale", dv::ConfigOption::intOption("Accelerometer scale configuration.", 1, 0, 3));
		config.add("imu/GyroFullScale", dv::ConfigOption::intOption("Gyroscope scale configuration.", 1, 0, 3));

		config.setPriorityOptions({"imu/RunAccelerometer", "imu/RunGyroscope"});
	}

	void imuConfigCreateDynamic(const struct caer_davis_info *devInfo) {
		if (devInfo->imuType == IMU_INVENSENSE_9250) {
			// InvenSense MPU 9250 IMU.
			config.add("imu/AccelDLPF",
				dv::ConfigOption::intOption("Accelerometer digital low-pass filter configuration.", 1, 0, 7));
			config.add("imu/GyroDLPF",
				dv::ConfigOption::intOption("Gyroscope digital low-pass filter configuration.", 1, 0, 7));
		}
		else {
			// InvenSense MPU 6050/6150 IMU.
			config.add("imu/DigitalLowPassFilter",
				dv::ConfigOption::intOption("Accelerometer/Gyroscope digital low-pass filter configuration.", 1, 0, 7));
		}
	}

	void imuConfigSend(const struct caer_davis_info *devInfo) {
		device.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER,
			static_cast<uint32_t>(config.getInt("imu/SampleRateDivider")));

		if (devInfo->imuType == IMU_INVENSENSE_9250) {
			device.configSet(
				DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_DLPF, static_cast<uint32_t>(config.getInt("imu/AccelDLPF")));
			device.configSet(
				DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_DLPF, static_cast<uint32_t>(config.getInt("imu/GyroDLPF")));
		}
		else {
			device.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER,
				static_cast<uint32_t>(config.getInt("imu/DigitalLowPassFilter")));
		}

		device.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE,
			static_cast<uint32_t>(config.getInt("imu/AccelFullScale")));
		device.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE,
			static_cast<uint32_t>(config.getInt("imu/GyroFullScale")));

		device.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, config.getBool("imu/RunAccelerometer"));
		device.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, config.getBool("imu/RunGyroscope"));
		device.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE, config.getBool("imu/RunTemperature"));
	}

	static void imuConfigListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		UNUSED_ARGUMENT(node);

		auto device = static_cast<libcaer::devices::davis *>(userData);

		std::string key{changeKey};

		if (event == DVCFG_ATTRIBUTE_MODIFIED) {
			if (changeType == DVCFG_TYPE_INT && key == "SampleRateDivider") {
				device->configSet(
					DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "DigitalLowPassFilter") {
				device->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER,
					static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "AccelDLPF") {
				device->configSet(
					DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_DLPF, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "AccelFullScale") {
				device->configSet(
					DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "GyroDLPF") {
				device->configSet(
					DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_DLPF, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "GyroFullScale") {
				device->configSet(
					DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "RunAccelerometer") {
				device->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "RunGyroscope") {
				device->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "RunTemperature") {
				device->configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE, changeValue.boolean);
			}
		}
	}

	static void externalInputConfigCreate(dv::RuntimeConfig &config) {
		// Subsystem 4: External Input
		config.add("externalInput/RunDetector", dv::ConfigOption::boolOption("Enable signal detector."));
		config.add("externalInput/DetectRisingEdges",
			dv::ConfigOption::boolOption("Emit special event if a rising edge is detected."));
		config.add("externalInput/DetectFallingEdges",
			dv::ConfigOption::boolOption("Emit special event if a falling edge is detected."));
		config.add(
			"externalInput/DetectPulses", dv::ConfigOption::boolOption("Emit special event if a pulse is detected."));
		config.add("externalInput/DetectPulsePolarity",
			dv::ConfigOption::boolOption("Polarity of the pulse to be detected.", true));
		config.add("externalInput/DetectPulseLength",
			dv::ConfigOption::intOption(
				"Minimal length of the pulse to be detected (in µs).", 10, 1, ((0x01 << 20) - 1)));

		config.setPriorityOptions({"externalInput/"});
	}

	void externalInputConfigCreateDynamic(const struct caer_davis_info *devInfo) {
		if (devInfo->extInputHasGenerator) {
			config.add(
				"externalInput/RunGenerator", dv::ConfigOption::boolOption("Enable signal generator (PWM-like)."));
			config.add("externalInput/GeneratePulsePolarity",
				dv::ConfigOption::boolOption("Polarity of the generated pulse.", true));
			config.add("externalInput/GeneratePulseInterval",
				dv::ConfigOption::intOption(
					"Time interval between consecutive pulses (in µs).", 10, 1, ((0x01 << 20) - 1)));
			config.add("externalInput/GeneratePulseLength",
				dv::ConfigOption::intOption("Time length of a pulse (in µs).", 5, 1, ((0x01 << 20) - 1)));
			config.add("externalInput/GenerateInjectOnRisingEdge",
				dv::ConfigOption::boolOption("Emit a special event when a rising edge is generated."));
			config.add("externalInput/GenerateInjectOnFallingEdge",
				dv::ConfigOption::boolOption("Emit a special event when a falling edge is generated."));
		}
	}

	void externalInputConfigSend(const struct caer_davis_info *devInfo) {
		device.configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES,
			config.getBool("externalInput/DetectRisingEdges"));
		device.configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES,
			config.getBool("externalInput/DetectFallingEdges"));
		device.configSet(
			DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES, config.getBool("externalInput/DetectPulses"));
		device.configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY,
			config.getBool("externalInput/DetectPulsePolarity"));
		device.configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH,
			static_cast<uint32_t>(config.getInt("externalInput/DetectPulseLength")));
		device.configSet(
			DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR, config.getBool("externalInput/RunDetector"));

		if (devInfo->extInputHasGenerator) {
			device.configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY,
				config.getBool("externalInput/GeneratePulsePolarity"));
			device.configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL,
				static_cast<uint32_t>(config.getInt("externalInput/GeneratePulseInterval")));
			device.configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH,
				static_cast<uint32_t>(config.getInt("externalInput/GeneratePulseLength")));
			device.configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE,
				config.getBool("externalInput/GenerateInjectOnRisingEdge"));
			device.configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE,
				config.getBool("externalInput/GenerateInjectOnFallingEdge"));
			device.configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR,
				config.getBool("externalInput/RunGenerator"));
		}
	}

	static void externalInputConfigListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		UNUSED_ARGUMENT(node);

		auto device = static_cast<libcaer::devices::davis *>(userData);

		std::string key{changeKey};

		if (event == DVCFG_ATTRIBUTE_MODIFIED) {
			if (changeType == DVCFG_TYPE_BOOL && key == "DetectRisingEdges") {
				device->configSet(
					DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "DetectFallingEdges") {
				device->configSet(
					DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "DetectPulses") {
				device->configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "DetectPulsePolarity") {
				device->configSet(
					DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_INT && key == "DetectPulseLength") {
				device->configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH,
					static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "RunDetector") {
				device->configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "GeneratePulsePolarity") {
				device->configSet(
					DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_INT && key == "GeneratePulseInterval") {
				device->configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL,
					static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "GeneratePulseLength") {
				device->configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH,
					static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "GenerateInjectOnRisingEdge") {
				device->configSet(
					DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "GenerateInjectOnFallingEdge") {
				device->configSet(
					DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE, changeValue.boolean);
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "RunGenerator") {
				device->configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR, changeValue.boolean);
			}
		}
	}

	static void usbConfigCreate(dv::RuntimeConfig &config) {
		// Subsystem 9: FX2/3 USB Configuration and USB buffer settings.
		config.add(
			"usb/Run", dv::ConfigOption::boolOption("Enable the USB state machine (FPGA to USB data exchange).", true));
		config.add("usb/EarlyPacketDelay",
			dv::ConfigOption::intOption(
				"Send early USB packets if this timeout is reached (in 125µs time-slices).", 8, 1, 8000));

		// USB buffer settings.
		config.add("usb/BufferNumber", dv::ConfigOption::intOption("Number of USB transfers.", 8, 2, 128));
		config.add("usb/BufferSize",
			dv::ConfigOption::intOption("Size in bytes of data buffers for USB transfers.", 8192, 512, 32768));

		config.setPriorityOptions({"usb/"});
	}

	void usbConfigSend() {
		device.configSet(CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_NUMBER,
			static_cast<uint32_t>(config.getInt("usb/BufferNumber")));
		device.configSet(CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_SIZE,
			static_cast<uint32_t>(config.getInt("usb/BufferSize")));

		device.configSet(DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_EARLY_PACKET_DELAY,
			static_cast<uint32_t>(config.getInt("usb/EarlyPacketDelay")));
		device.configSet(DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_RUN, config.getBool("usb/Run"));
	}

	static void usbConfigListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		UNUSED_ARGUMENT(node);

		auto device = static_cast<libcaer::devices::davis *>(userData);

		std::string key{changeKey};

		if (event == DVCFG_ATTRIBUTE_MODIFIED) {
			if (changeType == DVCFG_TYPE_INT && key == "BufferNumber") {
				device->configSet(
					CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_NUMBER, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "BufferSize") {
				device->configSet(
					CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_SIZE, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "EarlyPacketDelay") {
				device->configSet(
					DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_EARLY_PACKET_DELAY, static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_BOOL && key == "Run") {
				device->configSet(DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_RUN, changeValue.boolean);
			}
		}
	}

	static void systemConfigCreate(dv::RuntimeConfig &config) {
		// Packet settings (size (in events) and time interval (in µs)).
		config.add("system/PacketContainerMaxPacketSize",
			dv::ConfigOption::intOption("Maximum packet size in events, when any packet reaches this size, the "
										"EventPacketContainer is sent for processing.",
				0, 0, 10 * 1024 * 1024));
		config.add("system/PacketContainerInterval",
			dv::ConfigOption::intOption("Time interval in µs, each sent EventPacketContainer will span this interval.",
				10000, 1, 120 * 1000 * 1000));

		// Ring-buffer setting (only changes value on module init/shutdown cycles).
		config.add("system/DataExchangeBufferSize",
			dv::ConfigOption::intOption(
				"Size of EventPacketContainer queue, used for transfers between data acquisition thread and mainloop.",
				64, 8, 1024));

		config.setPriorityOptions({"system/"});
	}

	void systemConfigSend() {
		device.configSet(CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_PACKET_SIZE,
			static_cast<uint32_t>(config.getInt("system/PacketContainerMaxPacketSize")));
		device.configSet(CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL,
			static_cast<uint32_t>(config.getInt("system/PacketContainerInterval")));

		// Changes only take effect on module start!
		device.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BUFFER_SIZE,
			static_cast<uint32_t>(config.getInt("system/DataExchangeBufferSize")));
	}

	static void systemConfigListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		UNUSED_ARGUMENT(node);

		auto device = static_cast<libcaer::devices::davis *>(userData);

		std::string key{changeKey};
		if (event == DVCFG_ATTRIBUTE_MODIFIED) {
			if (changeType == DVCFG_TYPE_INT && key == "PacketContainerMaxPacketSize") {
				device->configSet(CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_PACKET_SIZE,
					static_cast<uint32_t>(changeValue.iint));
			}
			else if (changeType == DVCFG_TYPE_INT && key == "PacketContainerInterval") {
				device->configSet(CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL,
					static_cast<uint32_t>(changeValue.iint));
			}
		}
	}

	static void logLevelListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		UNUSED_ARGUMENT(node);

		auto device = static_cast<libcaer::devices::davis *>(userData);

		std::string key{changeKey};

		if (event == DVCFG_ATTRIBUTE_MODIFIED && changeType == DVCFG_TYPE_STRING && key == "logLevel") {
			device->configSet(CAER_HOST_CONFIG_LOG, CAER_HOST_CONFIG_LOG_LEVEL,
				static_cast<uint32_t>(dv::LoggerInternal::logLevelNameToInteger(changeValue.string)));
		}
	}

	static void modeListener(dvConfigNode node, void *userData, enum dvConfigAttributeEvents event,
		const char *changeKey, enum dvConfigAttributeType changeType, union dvConfigAttributeValue changeValue) {
		UNUSED_ARGUMENT(node);

		auto device = static_cast<libcaer::devices::davis *>(userData);

		std::string key{changeKey};

		if (event == DVCFG_ATTRIBUTE_MODIFIED && changeType == DVCFG_TYPE_STRING && key == "dataMode") {
			std::string value{changeValue.string};

			bool runDVS = (value.find("Events") != std::string::npos);
			device->configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, runDVS);

			bool runAPS = (value.find("Frames") != std::string::npos);
			device->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, runAPS);
		}
	}

	static union dvConfigAttributeValue statisticsUpdater(
		void *userData, const char *key, enum dvConfigAttributeType type) {
		UNUSED_ARGUMENT(type); // We know all statistics are always LONG.

		auto device = static_cast<libcaer::devices::davis *>(userData);

		std::string keyStr{key};

		union dvConfigAttributeValue statisticValue = {.ilong = 0};

		try {
			if (keyStr == "muxDroppedDVS") {
				device->configGet64(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_STATISTICS_DVS_DROPPED,
					reinterpret_cast<uint64_t *>(&statisticValue.ilong));
			}
			else if (keyStr == "muxDroppedExtInput") {
				device->configGet64(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_STATISTICS_EXTINPUT_DROPPED,
					reinterpret_cast<uint64_t *>(&statisticValue.ilong));
			}
			else if (keyStr == "dvsEventsRow") {
				device->configGet64(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_STATISTICS_EVENTS_ROW,
					reinterpret_cast<uint64_t *>(&statisticValue.ilong));
			}
			else if (keyStr == "dvsEventsColumn") {
				device->configGet64(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_STATISTICS_EVENTS_COLUMN,
					reinterpret_cast<uint64_t *>(&statisticValue.ilong));
			}
			else if (keyStr == "dvsEventsDropped") {
				device->configGet64(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_STATISTICS_EVENTS_DROPPED,
					reinterpret_cast<uint64_t *>(&statisticValue.ilong));
			}
			else if (keyStr == "dvsFilteredPixel") {
				device->configGet64(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_STATISTICS_FILTERED_PIXELS,
					reinterpret_cast<uint64_t *>(&statisticValue.ilong));
			}
			else if (keyStr == "dvsFilteredRate") {
				device->configGet64(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_STATISTICS_FILTERED_REFRACTORY_PERIOD,
					reinterpret_cast<uint64_t *>(&statisticValue.ilong));
			}
			else if (keyStr == "dvsFilteredNoise") {
				device->configGet64(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_STATISTICS_FILTERED_BACKGROUND_ACTIVITY,
					reinterpret_cast<uint64_t *>(&statisticValue.ilong));
			}
		}
		catch (const std::runtime_error &) {
			// Catch communication failures and ignore them.
		}

		return (statisticValue);
	}

	static union dvConfigAttributeValue apsExposureUpdater(
		void *userData, const char *key, enum dvConfigAttributeType type) {
		UNUSED_ARGUMENT(key);  // This is for the Exposure key only.
		UNUSED_ARGUMENT(type); // We know Exposure is always INT.

		auto device = static_cast<libcaer::devices::davis *>(userData);

		union dvConfigAttributeValue currentExposureValue = {.iint = 0};

		try {
			device->configGet(
				DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, reinterpret_cast<uint32_t *>(&currentExposureValue.iint));
		}
		catch (const std::runtime_error &) {
			// Catch communication failures and ignore them.
		}

		return (currentExposureValue);
	}

	static uint32_t mapFrameMode(const std::string &strVal) {
		if (strVal == "Default") {
			return (APS_FRAME_DEFAULT);
		}
		else if (strVal == "Grayscale") {
			return (APS_FRAME_GRAYSCALE);
		}
		else {
			return (APS_FRAME_ORIGINAL);
		}
	}

	static uint32_t mapAccelRange(const std::string &strVal) {
		if (strVal == "±2G") {
			return (ACCEL_2G);
		}
		else if (strVal == "±4G") {
			return (ACCEL_4G);
		}
		else if (strVal == "±8G") {
			return (ACCEL_8G);
		}
		else {
			return (ACCEL_16G);
		}
	}

	static uint32_t mapGyroRange(const std::string &strVal) {
		if (strVal == "±250°/s") {
			return (GYRO_250DPS);
		}
		else if (strVal == "±500°/s") {
			return (GYRO_500DPS);
		}
		else if (strVal == "±1000°/s") {
			return (GYRO_1000DPS);
		}
		else {
			return (GYRO_2000DPS);
		}
	}

	void createVDACBiasSetting(const std::string &biasPath, uint8_t voltageValue, uint8_t currentValue) {
		config.add(
			biasPath + "/voltageValue", dv::ConfigOption::intOption("Voltage, as a fraction of 1/64th of VDD=3.3V.",
											static_cast<int8_t>(voltageValue), 0, 63));
		config.add(biasPath + "/currentValue",
			dv::ConfigOption::intOption("Current that drives the voltage.", static_cast<int8_t>(currentValue), 0, 7));

		config.setPriorityOptions({biasPath + "/"});
	}

	uint16_t generateVDACBias(const std::string &biasPath) {
		return (generateVDACBias(moduleNode.getRelativeNode(biasPath + "/")));
	}

	static uint16_t generateVDACBias(dv::Config::Node biasNode) {
		// Build up bias value from all its components.
		struct caer_bias_vdac biasValue = {
			.voltageValue = static_cast<uint8_t>(biasNode.getInt("voltageValue")),
			.currentValue = static_cast<uint8_t>(biasNode.getInt("currentValue")),
		};

		return (caerBiasVDACGenerate(biasValue));
	}

	void createCoarseFineBiasSetting(const std::string &biasPath, uint8_t coarseValue, uint8_t fineValue, bool enabled,
		const std::string &sex, const std::string &type) {
		config.add(biasPath + "/coarseValue", dv::ConfigOption::intOption("Coarse current value (big adjustments).",
												  static_cast<int8_t>(coarseValue), 0, 7));
		config.add(biasPath + "/fineValue", dv::ConfigOption::intOption("Fine current value (small adjustments).",
												static_cast<int16_t>(fineValue), 0, 255));
		config.add(biasPath + "/enabled", dv::ConfigOption::boolOption("Bias enabled.", enabled));
		config.add(biasPath + "/sex", dv::ConfigOption::listOption("Bias sex.", sex, {"N", "P"}));
		config.add(biasPath + "/type", dv::ConfigOption::listOption("Bias type.", type, {"Normal", "Cascode"}));
		config.add(biasPath + "/currentLevel",
			dv::ConfigOption::listOption("Bias current level.", "Normal", {"Normal", "Low"}));

		config.setPriorityOptions({biasPath + "/"});
	}

	uint16_t generateCoarseFineBias(const std::string &biasPath) {
		return (generateCoarseFineBias(moduleNode.getRelativeNode(biasPath + "/")));
	}

	static uint16_t generateCoarseFineBias(dv::Config::Node biasNode) {
		// Build up bias value from all its components.
		auto sexString          = biasNode.getString("sex");
		auto typeString         = biasNode.getString("type");
		auto currentLevelString = biasNode.getString("currentLevel");

		struct caer_bias_coarsefine biasValue = {
			.coarseValue        = static_cast<uint8_t>(biasNode.getInt("coarseValue")),
			.fineValue          = static_cast<uint8_t>(biasNode.getInt("fineValue")),
			.enabled            = biasNode.getBool("enabled"),
			.sexN               = (sexString == "N"),
			.typeNormal         = (typeString == "Normal"),
			.currentLevelNormal = (currentLevelString == "Normal"),
		};

		return (caerBiasCoarseFineGenerate(biasValue));
	}

	void createShiftedSourceBiasSetting(const std::string &biasPath, uint8_t refValue, uint8_t regValue,
		const std::string &operatingMode, const std::string &voltageLevel) {
		config.add(biasPath + "/refValue",
			dv::ConfigOption::intOption("Shifted-source bias level.", static_cast<int8_t>(refValue), 0, 63));
		config.add(
			biasPath + "/regValue", dv::ConfigOption::intOption("Shifted-source bias current for buffer amplifier.",
										static_cast<int8_t>(regValue), 0, 63));
		config.add(biasPath + "/operatingMode", dv::ConfigOption::listOption("Shifted-source operating mode.",
													operatingMode, {"ShiftedSource", "HiZ", "TiedToRail"}));
		config.add(biasPath + "/voltageLevel", dv::ConfigOption::listOption("Shifted-source voltage level.",
												   voltageLevel, {"SplitGate", "SingleDiode", "DoubleDiode"}));

		config.setPriorityOptions({biasPath + "/"});
	}

	uint16_t generateShiftedSourceBias(const std::string &biasPath) {
		return (generateShiftedSourceBias(moduleNode.getRelativeNode(biasPath + "/")));
	}

	static uint16_t generateShiftedSourceBias(dv::Config::Node biasNode) {
		// Build up bias value from all its components.
		auto operatingModeString = biasNode.getString("operatingMode");
		auto voltageLevelString  = biasNode.getString("voltageLevel");

		struct caer_bias_shiftedsource biasValue = {
			.refValue      = static_cast<uint8_t>(biasNode.getInt("refValue")),
			.regValue      = static_cast<uint8_t>(biasNode.getInt("regValue")),
			.operatingMode = (operatingModeString == "HiZ")
								 ? (HI_Z)
								 : ((operatingModeString == "TiedToRail") ? (TIED_TO_RAIL) : (SHIFTED_SOURCE)),
			.voltageLevel = (voltageLevelString == "SingleDiode")
								? (SINGLE_DIODE)
								: ((voltageLevelString == "DoubleDiode") ? (DOUBLE_DIODE) : (SPLIT_GATE)),
		};

		return (caerBiasShiftedSourceGenerate(biasValue));
	}
};

registerModuleClass(davis)
