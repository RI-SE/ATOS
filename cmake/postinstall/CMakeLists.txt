
# Ensure linkage is reloaded after install
install(CODE "execute_process(COMMAND ldconfig)")

# Ensure service files are found by handler
install(CODE "execute_process(COMMAND systemctl daemon-reload)")
# TODO enable/disable and start/stop for systemctl

