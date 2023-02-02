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

#ifdef __cplusplus
}
#endif


#include "module.hpp"
#include "journalmodelcollection.hpp"
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

	JournalModelCollection journals;

	void onArmMessage(const ROSChannels::Arm::message_type::SharedPtr) override;
	void onStopMessage(const ROSChannels::Stop::message_type::SharedPtr) override;
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
  void onReplayMessage(const ROSChannels::Replay::message_type::SharedPtr) override;

  ROSChannels::Arm::Sub armSub;
  ROSChannels::Stop::Sub stopSub;
  ROSChannels::Abort::Sub abortSub;
  ROSChannels::Replay::Sub replaySub;
  ROSChannels::Exit::Sub exitSub;
};

#endif //__LOGGER_H_INCLUDED__
