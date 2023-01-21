#ifndef ATOSBASE_HPP
#define ATOSBASE_HPP

#include <memory>
#include <std_srvs/srv/set_bool.hpp>
#include "atos_interfaces/srv/get_object_ids.hpp"
#include "module.hpp"

class ATOSBase : public Module {
   public:
	ATOSBase();
	~ATOSBase();

   private:
	static inline std::string const moduleName = "ATOS_base";
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr initDataDictionaryService;
	rclcpp::Service<atos_interfaces::srv::GetObjectIds>::SharedPtr getObjectIdsService;

	void onExitMessage(const ROSChannels::Exit::message_type::SharedPtr) override;
	// Module only provides plumbing, no need to handle abort
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override{};

	void onInitDataDictionary(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
							  std::shared_ptr<std_srvs::srv::SetBool::Response>);
	void onRequestObjectIDs(const std::shared_ptr<atos_interfaces::srv::GetObjectIds::Request>,
							std::shared_ptr<atos_interfaces::srv::GetObjectIds::Response>);
	bool isInitialized = false;
	ROSChannels::Exit::Sub exitSub;
};

#endif