/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"

Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 4e-3, 4e-3, 4e2, 8e-3, 4.5e-2, 2 / 2 / M_PI;
    R.row(0) << 3e1, 7;
    R.row(1) << 7, 3e1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

void plot_trajectory(std::vector<float> x_history, std::vector<float> y_history, std::vector<float> theta_history, std::vector<float> time_history){

    matplot::figure();
    matplot::plot(time_history, x_history);
    matplot::title("Variable X to Y");
    
    
    matplot::figure();
    matplot::plot(time_history, y_history);
    matplot::title("Variable Y");    
    
    
    matplot::figure();
    matplot::plot(time_history, theta_history);
    matplot::title("Variable Theta");

    matplot::show();
}

void audioCallback(void* userdata, Uint8* stream, int len){




}

int main(int argc, char* args[])
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distX(0, SCREEN_WIDTH);
    std::uniform_int_distribution<int> distY(0, SCREEN_HEIGHT);
    int x_rd = distX(gen);
    int y_rd = distY(gen);
    auto resetWindow = [](SDL_Window* window) { if (window) SDL_DestroyWindow(window); };
    std::shared_ptr<SDL_Window> gWindow(nullptr, resetWindow);
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;

    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    initial_state << x_rd, y_rd, 0, 0, 0, 0;
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    Eigen::VectorXf working_goal_state = Eigen::VectorXf::Zero(6);
    Eigen::VectorXf current_state = Eigen::VectorXf::Zero(6);
    goal_state << SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.01;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;
    std::vector<float> time_history;

    Eigen::VectorXf state = quadrotor.GetState();



    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        

        /*
        if (SDL_Init(SDL_INIT_AUDIO) < 0)
        {
        SDL_Log("Failed to initialize SDL audio: %s", SDL_GetError());
        return 1;
        }


        SDL_AudioSpec desiredSpec, obtainedSpec;
        SDL_zero(desiredSpec);
        desiredSpec.freq = 44100;             // Sample rate (Hz)
        desiredSpec.format = AUDIO_S16SYS;    // Sample format (16-bit signed, system byte order)
        desiredSpec.channels = 2;             // Number of channels (stereo)
        desiredSpec.samples = 4096;           // Buffer size (number of samples)
        desiredSpec.callback = audioCallback;
*/


        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y, i=1;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEMOTION)
                {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse position: (" << x << ", " << y << ")" <<  "     "<<current_state[0]  <<"     "<< current_state[1]<< "     "<<current_state[2]<< std::endl;
                }
                else if(e.type == SDL_MOUSEBUTTONDOWN){
                    SDL_GetMouseState(&x, &y);
                    working_goal_state << x, y, 0, 0, 0, 0;
                    quadrotor.SetGoal(working_goal_state);
                }
                else if(e.type == SDL_KEYDOWN){
                    if(e.key.keysym.sym == SDLK_p){
                        quit = true;
                        gWindow.reset();
                        //plot_trajectory(x_history, y_history, theta_history, time_history);
                    }
                }
            }
            
            SDL_Delay((int) dt * 100);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            current_state=quadrotor.GetState();

            x_history.push_back(current_state(0));
            y_history.push_back(current_state(1));
            theta_history.push_back(current_state(2));
            time_history.push_back(dt*100*i);
            i++;

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
        }
    }
    SDL_Quit();

    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
