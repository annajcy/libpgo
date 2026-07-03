set(CGALINTERFACE_HEADERS cgalBasic.h cgalInterface.h cgalTemplateUtilities.h)
set(CGALINTERFACE_SOURCES cgalInterface.cpp)
set(CGALINTERFACE_DEPS pgoLogging eigenSupport mesh CGAL::CGAL CGAL::TBB_support)

pgo_require_targets(cgalInterface TARGETS ${CGALINTERFACE_DEPS})

add_libpgo_lib(cgalInterface "${CGALINTERFACE_SOURCES}" "${CGALINTERFACE_HEADERS}")

target_link_libraries(cgalInterface PUBLIC ${CGALINTERFACE_DEPS})

if (TARGET Ceres::ceres)
  target_link_libraries(cgalInterface PUBLIC Ceres::ceres)
endif()
