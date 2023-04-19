//
// Created by arun on 4/18/23.
//

#ifndef PARTICLE_FILTER_PARAMS_H
#define PARTICLE_FILTER_PARAMS_H

constexpr double sigma_hit = 0.1; // Used in beam measurement model P_hit calculation - Chapter 6 (6.3.1)
constexpr double lambda_short = 0.1; // Used in beam measurement model P_short calculation - Chapter 6 (6.3.1)
constexpr double z_hit = 0.25; // Used in beam measurement model weights calculation - Chapter 6 (6.3.1)
constexpr double z_short = 0.25; // Used in beam measurement model weights calculation - Chapter 6 (6.3.1)
constexpr double z_max = 0.25; // Used in beam measurement model weights calculation - Chapter 6 (6.3.1)
constexpr double z_rand = 0.25; // Used in beam measurement model weights calculation - Chapter 6 (6.3.1)


#endif //PARTICLE_FILTER_PARAMS_H
