/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2020 AstaZero
  ------------------------------------------------------------------------------
  -- File        : journalcontrol.h
  -- Author      : Lukas Wikander
  -- Description :
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __JOURNALCONTROL_H_INCLUDED__
#define __JOURNALCONTROL_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

#include "util.h"
#include "logging.h"

#ifdef __cplusplus
}
#endif


#include "module.hpp"
#include "journalcollection.hpp"
/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
class JournalControl : public Module
{
public:
	explicit JournalControl();
	void initialize();
private:
	static inline std::string const moduleName = "journal_control";

	JournalCollection journals;

	void onArmMessage(const Empty::SharedPtr) override;
	void onStopMessage(const Empty::SharedPtr) override;
	void onAbortMessage(const Empty::SharedPtr) override;
  void onReplayMessage(const Empty::SharedPtr) override;

  ROSChannels::Arm::Sub armSub;
  ROSChannels::Stop::Sub stopSub;
  ROSChannels::Abort::Sub abortSub;
  ROSChannels::Replay::Sub replaySub;
  ROSChannels::Exit::Sub exitSub;
};

#endif //__LOGGER_H_INCLUDED__
