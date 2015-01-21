add_module(ParticleFilterLocalisation
        ParticleFilterLocalisation.cpp
LINK_LIBRARIES
        DataObstacle
        AAData
)

add_module(DisplayParticles
        DisplayParticles.cpp
LINK_LIBRARIES
        Gui
)

