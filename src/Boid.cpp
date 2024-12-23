#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include "SFML/Graphics.hpp"
#include "Boid.h"

// Global Variables for borders()
// desktopTemp gets screen resolution of PC running the program
sf::VideoMode desktopTemp = sf::VideoMode::getDesktopMode();
const int window_height = desktopTemp.size.y;
const int window_width = desktopTemp.size.x;

#define w_height window_height
#define w_width window_width
#define PI 3.141592635

// =============================================== //
// ======== Boid Functions from Boid.h =========== //
// =============================================== //

namespace boids {

Boid::Boid(float x, float y)
{
    velocity = Vector2f(rand()%3 - 2, rand()%3 - 2);
    location = Vector2f(x, y);
    maxSpeed = 3.5;
    maxForce = 0.5;
}

Boid::Boid(float x, float y, bool predCheck)
{
    predator = predCheck;
    if (predCheck == true) {
        maxSpeed = 7.5;
        maxForce = 0.5;
        velocity = Vector2f(rand()%3 - 1, rand()%3 - 1);
    } else {
        maxSpeed = 3.5;
        maxForce = 0.5;
        velocity = Vector2f(rand()%3 - 2, rand()%3 - 2);
    }
    location = Vector2f(x, y);
}

// Adds force Pvector to current force Pvector
void Boid::applyForce(const Vector2f& force)
{
    acceleration += force;
}

// Separation
// Keeps boids from getting too close to one another
Vector2f Boid::Separation(const std::vector<Boid>& boids)
{
    // Distance of field of vision for separation between boids
    float desiredseparation = 20;
    Vector2f steer(0, 0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (int i = 0; i < boids.size(); i++) {
        // Calculate distance from current boid to boid we're looking at
        float d = location.distance(boids[i].location);
        // If this is a fellow boid and it's too close, move away from it
        if ((d > 0) && (d < desiredseparation)) {
            Vector2f diff = location - boids[i].location;
            diff.normalize();
            diff /= d;
            steer += diff;
            count++;
        }
        // If current boid is a predator and the boid we're looking at is also
        // a predator, then separate only slightly
        if ((d > 0) && (d < desiredseparation) && predator == true
            && boids[i].predator == true) {
            Vector2f pred2pred = location - boids[i].location;
            pred2pred.normalize();
            pred2pred /= d;
            steer += pred2pred;
            count++;
        }
        // If current boid is not a predator, but the boid we're looking at is
        // a predator, then create a large separation Pvector
        else if ((d > 0) && (d < desiredseparation+70) && boids[i].predator == true) {
            Vector2f pred = location - boids[i].location;
            pred *= 900.0f;
            steer += pred;
            count++;
        }
    }
    // Adds average difference of location to acceleration
    if (count > 0)
        steer /= static_cast<float>(count);
    if (steer.magnitude() > 0) {
        // Steering = Desired - Velocity
        steer.normalize();
        steer *= maxSpeed;
        steer -= velocity;
        steer.limit(maxForce);
    }
    return steer;
}

// Alignment
// Calculates the average velocity of boids in the field of vision and
// manipulates the velocity of the current boid in order to match it
Vector2f Boid::Alignment(const std::vector<Boid>& Boids)
{
    float neighbordist = 50; // Field of vision

    Vector2f sum;
    int count = 0;
    for (int i = 0; i < Boids.size(); i++) {
        float d = location.distance(Boids[i].location);
        if ((d > 0) && (d < neighbordist)) { // 0 < d < 50
            sum += Boids[i].velocity;
            count++;
        }
    }
    // If there are boids close enough for alignment...
    if (count > 0) {
        sum /= static_cast<float>(count);// Divide sum by the number of close boids (average of velocity)
        sum.normalize();            // Turn sum into a unit vector, and
        sum *= maxSpeed;    // Multiply by maxSpeed
        // Steer = Desired - Velocity
        Vector2f steer = sum - velocity;
        steer.limit(maxForce);
        return steer;
    } else {
        return {};
    }
}

// Cohesion
// Finds the average location of nearby boids and manipulates the
// steering force to move in that direction.
Vector2f Boid::Cohesion(const std::vector<Boid>& Boids)
{
    float neighbordist = 50;
    Vector2f sum;
    int count = 0;
    for (int i = 0; i < Boids.size(); i++) {
        float d = location.distance(Boids[i].location);
        if ((d > 0) && (d < neighbordist)) {
            sum += Boids[i].location;
            count++;
        }
    }
    if (count > 0) {
        sum /= static_cast<float>(count);
        return seek(sum);
    } else {
        return {};
    }
}

// Limits the maxSpeed, finds necessary steering force and
// normalizes vectors
Vector2f Boid::seek(const Vector2f& v)
{
    Vector2f desired = -v;
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired *= maxSpeed;
    // Steering = Desired minus Velocity
    // acceleration.subTwoVector(desired, velocity);
    // acceleration.limit(maxForce);  // Limit to maximum steering force
    // return acceleration;
    // Pvector steer = desired - velocity;
    // steer.limit(maxForce);
    // return steer;
    return acceleration;
}

// Modifies velocity, location, and resets acceleration with values that
// are given by the three laws.
void Boid::update()
{
    //To make the slow down not as abrupt
    acceleration *= .4f;
    // Update velocity
    velocity += acceleration;
    // Limit speed
    velocity.limit(maxSpeed);
    location += velocity;
    // Reset accelertion to 0 each cycle
    acceleration = {};
}

// Run flock() on the flock of boids.
// This applies the three rules, modifies velocities accordingly, updates data,
// and corrects boids which are sitting outside of the SFML window
void Boid::run(const std::vector <Boid>& v)
{
    flock(v);
    update();
    borders();
}

// Applies the three laws to the flock of boids
void Boid::flock(const std::vector<Boid>& v)
{
    Vector2f sep = Separation(v);
    Vector2f ali = Alignment(v);
    Vector2f coh = Cohesion(v);
    // Arbitrarily weight these forces
    sep *= 1.5f;
    ali *= 1.0f; // Might need to alter weights for different characteristics
    coh *= 1.0f;
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
}

// Checks if boids go out of the window and if so, wraps them around to
// the other side.
void Boid::borders()
{
    if (location.x < 0)    location.x += w_width;
    if (location.y < 0)    location.y += w_height;
    if (location.x > 1000) location.x -= w_width;
    if (location.y > 1000) location.y -= w_height;
}

// Calculates the angle for the velocity of a boid which allows the visual
// image to rotate in the direction that it is going in.
float Boid::angle(const Vector2f& v)
{
    // From the definition of the dot product
    float angle = (float)(atan2(v.x, -v.y) * 180 / PI);
    return angle;
}

} // namespace boids