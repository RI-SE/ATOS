
project(atos_gui)

add_custom_target(${PROJECT_NAME} ALL)
add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	COMMAND npm install
	COMMAND npx generate-ros-messages #this command does not work in CMAKE_CURRENT_BINARY_DIR for some reason..
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION ${CMAKE_INSTALL_SYSCONFDIR})
