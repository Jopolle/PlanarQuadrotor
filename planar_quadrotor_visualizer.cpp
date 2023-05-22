#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}


/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta, x_1, x_2, y_1, y_2, x_3, y_3, x_11, y_11, x_22, y_22;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];


    int L = 50;
    int L2 = 40;
    int T = 15;
    int H = 20; 


    x_1 = q_x - cos(q_theta)*L;
    y_1 = q_y + sin(q_theta)*L;

    x_2 = q_x + cos(q_theta)*L;
    y_2 = q_y - sin(q_theta)*L;

    x_11 = q_x - cos(q_theta)*L2;
    y_11 = q_y + sin(q_theta)*L2;

    x_22 = q_x + cos(q_theta)*L2;
    y_22 = q_y - sin(q_theta)*L2;



    

    
    thickLineRGBA(gRenderer.get(), x_11, y_11, x_11, y_11-H, 8, 0x00, 0x00, 0xAA, 0xFF);
    thickLineRGBA(gRenderer.get(), x_22, y_22, x_22, y_22-H, 8, 0x00, 0x00, 0xAA, 0xFF);
    thickLineRGBA(gRenderer.get(), x_2, y_2, x_1, y_1, 14, 0x69, 0x69, 0x69, 0xFF);

    thickLineRGBA(gRenderer.get(), x_22+T, y_22-H, x_22-T, y_22-H, 5, 0x00, 0x00, 0x00, 0xFF);
    thickLineRGBA(gRenderer.get(), x_11+T, y_11-H, x_11-T, y_11-H, 5, 0x00, 0x00, 0x00, 0xFF);
}