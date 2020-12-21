// Copyright 2020 iniVation AG
#ifndef LOG_HPP_
#define LOG_HPP_

#include "dv-sdk/utils.h"

#include <array>
#include <atomic>
#include <boost/algorithm/string/join.hpp>
#include <string>
#include <string_view>

#define DV_LOG_FILE_NAME ".dv-logger.txt"

namespace dv::LoggerInternal {

static std::array<const std::string, 4> logLevelNames = {"ERROR", "WARNING", "INFO", "DEBUG"};

static inline std::string logLevelNamesCommaList() {
	return (boost::algorithm::join(logLevelNames, ","));
}

static inline dv::logLevel logLevelNameToEnum(const std::string &ls) {
	if (ls == logLevelNames[0]) {
		return (dv::logLevel::ERROR);
	}
	else if (ls == logLevelNames[1]) {
		return (dv::logLevel::WARNING);
	}
	else if (ls == logLevelNames[2]) {
		return (dv::logLevel::INFO);
	}
	else if (ls == logLevelNames[3]) {
		return (dv::logLevel::DEBUG);
	}
	else {
		// Old log-levels, handled here for backwards compatibility.
		if ((ls == "EMERGENCY") || (ls == "ALERT") || (ls == "CRITICAL")) {
			return (dv::logLevel::ERROR);
		}
		else {
			// Must be old log-level "NOTICE" or default fall-back.
			return (dv::logLevel::INFO);
		}
	}
}

static inline std::string logLevelIntegerToName(const int li) {
	switch (li) {
		case 0: // Old EMERGENCY.
		case 1: // Old ALERT.
		case 2: // Old CRITICAL.
		case DVLOG_ERROR:
			return logLevelNames[0];

		case DVLOG_WARNING:
			return logLevelNames[1];

		case 5: // Old NOTICE.
		case DVLOG_INFO:
		default:
			return logLevelNames[2];

		case DVLOG_DEBUG:
			return logLevelNames[3];
	}
}

static inline std::string logLevelEnumToName(const dv::logLevel li) {
	auto enumValue = static_cast<std::underlying_type_t<dv::logLevel>>(li);

	return logLevelIntegerToName(enumValue);
}

static inline int logLevelNameToInteger(const std::string &ls) {
	return (static_cast<std::underlying_type_t<dv::logLevel>>(logLevelNameToEnum(ls)));
}

struct LogBlock {
	std::string logPrefix;
	std::atomic<dv::logLevel> logLevel;
};

void Set(const LogBlock *logger);
const LogBlock *Get();

void Init();
void DisableConsoleOutput();

void LogInternal(dv::logLevel level, std::string_view message);

} // namespace dv::LoggerInternal

#endif /* LOG_HPP_ */
