set(target CnoidROSPlugin)

cnoid_make_gettext_mofiles(${target} mofiles)

add_library(${target} SHARED
  ROSPlugin.cpp
  BodyPublisherItem.cpp
  ${mofiles}
  )

target_link_libraries(${target}
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES}
  )

set_target_properties(${target} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR})

install(TARGETS ${target}
  LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}
  )

