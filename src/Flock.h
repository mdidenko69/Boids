#ifndef FLOCK_H_
#define FLOCK_H_

#include <iostream>
#include <vector>
#include "Boid.h"

namespace boids {
// Brief description of Flock Class:
// This file contains the class needed to create a flock of boids. It utilizes
// the boids class and initializes boid flocks with parameters that can be
// specified. This class will be utilized in main.

class Flock {
public:
    //Constructors
    Flock() {}
    // Accessor functions
    int getSize();
    // Read/write methods.
    Boid &getBoid(int i);
    // Mutator Functions
    void addBoid(const Boid& b);
    void flocking();
private:
    std::vector<Boid> flock;
};

} // namespace boids

#endif
