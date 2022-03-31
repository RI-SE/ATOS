#ifndef MAESTROBASE_HPP
#define MAESTROBASE_HPP

#include <memory>
#include <std_srvs/srv/set_bool.hpp>
#include "maestro_interfaces/srv/get_object_ids.hpp"
#include "module.hpp"

class MaestroBase : public Module {
   public:
	MaestroBase();
	~MaestroBase();
	void initialize();

   private:
	static inline std::string const moduleName = "maestro_base";
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr initDataDictionaryService;
	rclcpp::Service<maestro_interfaces::srv::GetObjectIds>::SharedPtr getObjectIdsService;

	void onExitMessage(const Empty::SharedPtr) override;
	// Module only provides plumbing, no need to handle abort
	void onAbortMessage(const Empty::SharedPtr) override{};

	void onInitDataDictionary(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
							  std::shared_ptr<std_srvs::srv::SetBool::Response>);
	void onRequestObjectIDs(const std::shared_ptr<maestro_interfaces::srv::GetObjectIds::Request>,
							std::shared_ptr<maestro_interfaces::srv::GetObjectIds::Response>);
	bool isInitialized = false;
	ROSChannels::Exit::Sub exitSub;
};

#endif