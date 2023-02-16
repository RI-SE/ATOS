/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/* -----------------------------------------------------------------------------
  -- File        : journalcontrol.cpp
  -- Author      : Lukas Wikander
  -- Description :
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

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
#include "journal.hpp"
#include "datadictionary.h"

#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
namespace fs = std::filesystem;
#else
namespace fs = std::experimental::filesystem;
#endif
using namespace ROSChannels;
using std::placeholders::_1;

static void signalHandler(int signo);

JournalControl::JournalControl()
	: Module(JournalControl::moduleName),
	journals(get_logger()),
	armSub(*this, std::bind(&JournalControl::onArmMessage, this, _1)),
	stopSub(*this, std::bind(&JournalControl::onStopMessage, this, _1)),
	abortSub(*this, std::bind(&JournalControl::onAbortMessage, this, _1)),
	replaySub(*this, std::bind(&JournalControl::onReplayMessage, this, _1)),
	exitSub(*this, std::bind(&JournalControl::onExitMessage, this, _1))
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

void JournalControl::onArmMessage(const Arm::message_type::SharedPtr msg)
{
	journals.placeStartBookmarks();
}

void JournalControl::onStopMessage(const Stop::message_type::SharedPtr msg)
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

void JournalControl::onAbortMessage(const Abort::message_type::SharedPtr msg)
{
	// Temporary: Treat ABORT as stop signal
	onStopMessage(msg);
}

void JournalControl::onReplayMessage(const Replay::message_type::SharedPtr msg)
{
	RCLCPP_WARN(get_logger(), "Replay function out of date");
}

void Module::onExitMessage(const Exit::message_type::SharedPtr){
    this->quit=true;
}

void signalHandler(int signo) {
	rclcpp::shutdown();
}