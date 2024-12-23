#ifndef GAME_H
#define GAME_H

#include <iostream>
#include "Flock.h"
#include "Boid.h"
#include "SFML/Window.hpp"
#include "SFML/Graphics.hpp"


namespace boids {

// Game handles the instantiation of a flock of boids, game input, asks the
// model to compute the next step in the stimulation, and handles all of the
// program's interaction with SFML. 

class Game {
private:
    sf::RenderWindow window;
    unsigned int window_width;
    unsigned int window_height;

    Flock flock;
    float boidsSize;
    std::vector<sf::CircleShape> shapes;

    void Render();
    void HandleInput();

public:
    Game();
    void Run();
};

} // namespace boids

#endif
