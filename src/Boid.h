#ifndef BOID_H_
#define BOID_H_

#include "vector2.h"
#include <vector>
#include <stdlib.h>
#include <iostream>


// The Boid Class
//
// Attributes
//  bool predator: flag that specifies whether a given boid is a predator.
//  Pvector location: Vector that specifies a boid's location.
//  Pvector velocity: Vector that specifies a boid's current velocity.
//  Pvector acceleration: Vector that specifies a boid's current acceleration.
//  float maxSpeed: Limits magnitude of velocity vector.
//  float maxForce: Limits magnitude of acceleration vector. (F = m*a!)
//
// Methods
//  applyForce(Pvector force): Adds the given vector to acceleration
//
//  Pvector Separation(vector<Boid> Boids): If any other boids are within a
//      given distance, Separation computes a a vector that distances the
//      current boid from the boids that are too close.
//
//  Pvector Alignment(vector<Boid> Boids): Computes a vector that causes the
//      velocity of the current boid to match that of boids that are nearby.
//
//  Pvector Cohesion(vector<Boid> Boids): Computes a vector that causes the
//      current boid to seek the center of mass of nearby boids.

namespace boids {

class Boid {
public:
    bool predator;
    Vector2f location;
    Vector2f velocity;
    Vector2f acceleration;
    float maxSpeed;
    float maxForce;
    Boid() {}
    Boid(float x, float y);
    Boid(float x, float y, bool predCheck);
    void applyForce(const Vector2f& force);
    // Three Laws that boids follow
    Vector2f Separation(const std::vector<Boid>& Boids);
    Vector2f Alignment(const std::vector<Boid>& Boids);
    Vector2f Cohesion(const std::vector<Boid>& Boids);
    //Functions involving SFML and visualisation linking
    Vector2f seek(const Vector2f& v);
    void run(const std::vector<Boid>& v);
    void update();
    void flock(const std::vector<Boid>& v);
    void borders();
    float angle(const Vector2f& v);
};

} // namespace boids

#endif
