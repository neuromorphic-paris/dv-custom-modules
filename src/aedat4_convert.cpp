#include "aedat4_convert.hpp"

#define DV_API_OPENCV_SUPPORT 0

#include "dv-sdk/data/event.hpp"
#include "dv-sdk/data/frame.hpp"
#include "dv-sdk/data/imu.hpp"
#include "dv-sdk/data/trigger.hpp"

#include <libcaercpp/events/frame.hpp>
#include <libcaercpp/events/imu6.hpp>
#include <libcaercpp/events/polarity.hpp>
#include <libcaercpp/events/special.hpp>

void dvConvertToAedat4(caerEventPacketHeaderConst oldPacket, dvModuleData moduleData) {
	if (oldPacket == nullptr || moduleData == nullptr) {
		return;
	}

	if (caerEventPacketHeaderGetEventValid(oldPacket) <= 0) {
		// No valid events, nothing to do.
		return;
	}

	switch (caerEventPacketHeaderGetEventType(oldPacket)) {
		case POLARITY_EVENT: {
			auto newObject      = dvModuleOutputAllocate(moduleData, "events");
			auto newEventPacket = static_cast<dv::EventPacket *>(newObject->obj);

			const libcaer::events::PolarityEventPacket oldPacketPolarity(
				const_cast<caerEventPacketHeader>(oldPacket), false);

			newEventPacket->elements.reserve(static_cast<size_t>(oldPacketPolarity.getEventValid()));

			for (const auto &evt : oldPacketPolarity) {
				if (!evt.isValid()) {
					continue;
				}

				newEventPacket->elements.emplace_back(
					evt.getTimestamp64(oldPacketPolarity), evt.getX(), evt.getY(), evt.getPolarity());
			}

			if (newEventPacket->elements.size() > 0) {
				dvModuleOutputCommit(moduleData, "events");
			}

			break;
		}

		case FRAME_EVENT: {
			const libcaer::events::FrameEventPacket oldPacketFrame(const_cast<caerEventPacketHeader>(oldPacket), false);

			for (const auto &evt : oldPacketFrame) {
				if (!evt.isValid()) {
					continue;
				}

				auto newObject = dvModuleOutputAllocate(moduleData, "frames");
				auto newFrame  = static_cast<dv::Frame *>(newObject->obj);

				newFrame->timestamp                = evt.getTimestamp64(oldPacketFrame);
				newFrame->timestampStartOfFrame    = evt.getTSStartOfFrame64(oldPacketFrame);
				newFrame->timestampStartOfExposure = evt.getTSStartOfExposure64(oldPacketFrame);
				newFrame->timestampEndOfExposure   = evt.getTSEndOfExposure64(oldPacketFrame);
				newFrame->timestampEndOfFrame      = evt.getTSEndOfFrame64(oldPacketFrame);

				newFrame->sizeX     = static_cast<int16_t>(evt.getLengthX());
				newFrame->sizeY     = static_cast<int16_t>(evt.getLengthY());
				newFrame->positionX = static_cast<int16_t>(evt.getPositionX());
				newFrame->positionY = static_cast<int16_t>(evt.getPositionY());

				// New frame format specification.
				if (evt.getChannelNumber() == libcaer::events::FrameEvent::colorChannels::RGB) {
					// RGB to BGR.
					newFrame->format = dv::FrameFormat::BGR;
				}
				else if (evt.getChannelNumber() == libcaer::events::FrameEvent::colorChannels::RGBA) {
					// RGBA to BGRA.
					newFrame->format = dv::FrameFormat::BGRA;
				}
				else {
					// Default: grayscale.
					newFrame->format = dv::FrameFormat::GRAY;
				}

				newFrame->pixels.resize(evt.getPixelsMaxIndex());

				for (size_t px = 0; px < evt.getPixelsMaxIndex();) {
					switch (newFrame->format) {
						case dv::FrameFormat::GRAY:
							newFrame->pixels[px] = static_cast<uint8_t>(evt.getPixelArrayUnsafe()[px] >> 8);
							px += 1;
							break;

						case dv::FrameFormat::BGR:
							newFrame->pixels[px + 0] = static_cast<uint8_t>(evt.getPixelArrayUnsafe()[px + 2] >> 8);
							newFrame->pixels[px + 1] = static_cast<uint8_t>(evt.getPixelArrayUnsafe()[px + 1] >> 8);
							newFrame->pixels[px + 2] = static_cast<uint8_t>(evt.getPixelArrayUnsafe()[px + 0] >> 8);
							px += 3;
							break;

						case dv::FrameFormat::BGRA:
							newFrame->pixels[px + 0] = static_cast<uint8_t>(evt.getPixelArrayUnsafe()[px + 2] >> 8);
							newFrame->pixels[px + 1] = static_cast<uint8_t>(evt.getPixelArrayUnsafe()[px + 1] >> 8);
							newFrame->pixels[px + 2] = static_cast<uint8_t>(evt.getPixelArrayUnsafe()[px + 0] >> 8);
							newFrame->pixels[px + 3] = static_cast<uint8_t>(evt.getPixelArrayUnsafe()[px + 3] >> 8);
							px += 4;
							break;
					}
				}

				if (newFrame->pixels.size() > 0) {
					dvModuleOutputCommit(moduleData, "frames");
				}
			}

			break;
		}

		case IMU6_EVENT: {
			auto newObject    = dvModuleOutputAllocate(moduleData, "imu");
			auto newIMUPacket = static_cast<dv::IMUPacket *>(newObject->obj);

			const libcaer::events::IMU6EventPacket oldPacketIMU(const_cast<caerEventPacketHeader>(oldPacket), false);

			newIMUPacket->elements.reserve(static_cast<size_t>(oldPacketIMU.getEventValid()));

			for (const auto &evt : oldPacketIMU) {
				if (!evt.isValid()) {
					continue;
				}

				dv::IMU imu{};
				imu.timestamp      = evt.getTimestamp64(oldPacketIMU);
				imu.temperature    = evt.getTemp();
				imu.accelerometerX = evt.getAccelX();
				imu.accelerometerY = evt.getAccelY();
				imu.accelerometerZ = evt.getAccelZ();
				imu.gyroscopeX     = evt.getGyroX();
				imu.gyroscopeY     = evt.getGyroY();
				imu.gyroscopeZ     = evt.getGyroZ();

				newIMUPacket->elements.push_back(imu);
			}

			if (newIMUPacket->elements.size() > 0) {
				dvModuleOutputCommit(moduleData, "imu");
			}

			break;
		}

		case SPECIAL_EVENT: {
			auto newObject        = dvModuleOutputAllocate(moduleData, "triggers");
			auto newTriggerPacket = static_cast<dv::TriggerPacket *>(newObject->obj);

			const libcaer::events::SpecialEventPacket oldPacketSpecial(
				const_cast<caerEventPacketHeader>(oldPacket), false);

			newTriggerPacket->elements.reserve(static_cast<size_t>(oldPacketSpecial.getEventValid()));

			for (const auto &evt : oldPacketSpecial) {
				if (!evt.isValid()) {
					continue;
				}

				dv::Trigger trigger{};

				if (evt.getType() == TIMESTAMP_RESET) {
					trigger.type = dv::TriggerType::TIMESTAMP_RESET;
				}
				else if (evt.getType() == EXTERNAL_INPUT_RISING_EDGE) {
					trigger.type = dv::TriggerType::EXTERNAL_SIGNAL_RISING_EDGE;
				}
				else if (evt.getType() == EXTERNAL_INPUT_FALLING_EDGE) {
					trigger.type = dv::TriggerType::EXTERNAL_SIGNAL_FALLING_EDGE;
				}
				else if (evt.getType() == EXTERNAL_INPUT_PULSE) {
					trigger.type = dv::TriggerType::EXTERNAL_SIGNAL_PULSE;
				}
				else if (evt.getType() == EXTERNAL_GENERATOR_RISING_EDGE) {
					trigger.type = dv::TriggerType::EXTERNAL_GENERATOR_RISING_EDGE;
				}
				else if (evt.getType() == EXTERNAL_GENERATOR_FALLING_EDGE) {
					trigger.type = dv::TriggerType::EXTERNAL_GENERATOR_FALLING_EDGE;
				}
				else if (evt.getType() == APS_FRAME_START) {
					trigger.type = dv::TriggerType::APS_FRAME_START;
				}
				else if (evt.getType() == APS_FRAME_END) {
					trigger.type = dv::TriggerType::APS_FRAME_END;
				}
				else if (evt.getType() == APS_EXPOSURE_START) {
					trigger.type = dv::TriggerType::APS_EXPOSURE_START;
				}
				else if (evt.getType() == APS_EXPOSURE_END) {
					trigger.type = dv::TriggerType::APS_EXPOSURE_END;
				}
				else {
					continue;
				}

				trigger.timestamp = evt.getTimestamp64(oldPacketSpecial);

				newTriggerPacket->elements.push_back(trigger);
			}

			if (newTriggerPacket->elements.size() > 0) {
				dvModuleOutputCommit(moduleData, "triggers");
			}

			break;
		}

		default:
			// Unknown data.
			break;
	}
}
