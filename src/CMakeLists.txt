set(target CnoidROSPlugin)

cnoid_make_gettext_mofiles(${target} mofiles)

add_cnoid_plugin(${target} SHARED
  ROSPlugin.cpp
  BodyPublisherItem.cpp
  ${mofiles}
  )

target_link_libraries(${target}
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES}
  )

install(TARGETS ${target}
  LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}
  )
