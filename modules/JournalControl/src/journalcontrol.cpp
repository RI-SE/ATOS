/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2020 AstaZero
  ------------------------------------------------------------------------------
  -- File        : journalcontrol.cpp
  -- Author      : Lukas Wikander
  -- Description :
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#include "logging.h"
#include <signal.h>
#include <vector>
#include <unordered_set>
#include <set>
#include <fstream>
#include <iostream>
#include <iterator>
#include <chrono>
#include <algorithm>
// GCC version 8.1 brings non-experimental support for std::filesystem
#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

#include "journalcontrol.hpp"
#include "journal.h"
#include "datadictionary.h"

#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
namespace fs = std::filesystem;
#else
namespace fs = std::experimental::filesystem;
#endif
static void signalHandler(int signo);

JournalControl::JournalControl()
	: Module(JournalControl::moduleName),
	journals(get_logger()),
	armSub(*this, std::bind(&JournalControl::onArmMessage, this, std::placeholders::_1)),
	stopSub(*this, std::bind(&JournalControl::onStopMessage, this, std::placeholders::_1)),
	abortSub(*this, std::bind(&JournalControl::onAbortMessage, this, std::placeholders::_1)),
	replaySub(*this, std::bind(&JournalControl::onReplayMessage, this, std::placeholders::_1)),
	exitSub(*this, std::bind(&JournalControl::onExitMessage, this, std::placeholders::_1))
{
	initialize();
}

void JournalControl::initialize()
{
	int retval = 0;
	RCLCPP_INFO(get_logger(), "%s task running with PID: %d", get_name(), getpid());

	if (std::signal(SIGINT, signalHandler) == SIG_ERR) {
		throw std::runtime_error("Failed to register signal handler");
	}
}

void JournalControl::onArmMessage(const Empty::SharedPtr msg)
{
	journals.placeStartBookmarks();
}

void JournalControl::onStopMessage(const Empty::SharedPtr msg)
{
	try {
		// Save stop references
		journals.placeStopBookmarks();
		// If any additional journals were created in the start-stop interval,
		// insert them
		journals.insertNonBookmarked();
		// Merge journals into named output
		journals.dumpToFile();
	} catch (std::exception &e) {
		RCLCPP_ERROR(get_logger(), "Failed to save journal: %s", e.what());
	}
}

void JournalControl::onAbortMessage(const Empty::SharedPtr msg)
{
	// Temporary: Treat ABORT as stop signal
	onStopMessage(msg);
}

void JournalControl::onReplayMessage(const Empty::SharedPtr msg)
{
	LogMessage(LOG_LEVEL_WARNING, "Replay function out of date");
}

void signalHandler(int signo) {
	if (signo == SIGINT) {
		LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
		rclcpp::shutdown();
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
	}
}