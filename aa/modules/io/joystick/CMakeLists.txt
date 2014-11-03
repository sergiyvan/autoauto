PKG_SEARCH_MODULE(SDL sdl)

IF(SDL_FOUND)
	add_module(
		Joystick 
		Joystick.cpp
	TEST_SOURCES testJoystick.cpp 
	LINK_LIBRARIES ${SDL_LIBRARIES} EgoState CarState StateMachine
	TEST_LINK_LIBRARIES ${SDL_LIBRARIES} Archives EgoState CarState)

ENDIF(SDL_FOUND)
