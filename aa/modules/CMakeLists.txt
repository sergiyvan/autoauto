add_subdirectory(display)
set_property(GLOBAL PROPERTY "AUTO_MODULE" ON)
add_subdirectory(io)
add_subdirectory(models)
add_subdirectory(nav)
add_subdirectory(vision)
set_property(GLOBAL PROPERTY "AUTO_MODULE" OFF)

