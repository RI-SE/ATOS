
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "lukas.wikander@astazero.com")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libeigen3-dev (>=3.0.0), libsystemd-dev, libprotobuf-dev (>=3.0.0)")
set(CPACK_PACKAGE_VENDOR "AstaZero")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "AstaZero test track execution software")
set(CPACL_PACKAGE_DESCRIPTION_FILE "CPack.MaestroDescription.txt")

set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/postinst" "${CMAKE_CURRENT_SOURCE_DIR}/postrm")
include(CPack)
