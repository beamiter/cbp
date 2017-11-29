#include "Optimizer.h"
#include "settings.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
using CppAD::AD;

#include <eigen3/Eigen/Dense>

using Eigen::VectorXd;

static const size_t x_start = 0;
static const size_t y_start = 1;
static const size_t CS0 = 0;
static const size_t CS1 = 1;
static const size_t CS2 = 2;
static const size_t SIZEOF_POINT = 2;
static const size_t SIZEOF_CONSTRAINT = 2;
static const size_t SIZEOF_OBSTACLE = 2;

namespace 
{
     using CppAD::AD;

     class FG_eval 
     {
     public:

        /** @brief Basic scalar value type. */
        typedef AD<double> Scalar;

        /** @brief Differentiable variable ADvector type. */
        typedef CPPAD_TESTVECTOR( Scalar ) ADvector;

        void operator()(ADvector& fg, const ADvector& x)
        {    
            //assert( fg.size() == 3 );
            //assert( x.size()  == 4 );

            size_t i = 0;

            ADvector k(3);
            for (int i = 0; i < 3; ++i)
            {       
                k[i] = i + 1;
            }

            // fg[0] = (x1 - k[0])*(x1 - k[0]) + (x2 - k[1])*(x2 - k[1]) + (x3 - k[2])*(x3 - k[2]);

            // f value
            for (i = 0; i < 3; ++i)
            {
                fg[0] += CppAD::pow(x[i] - 0.75, 2);
            }

            //g value
            for (i = 0 ; i < 2; ++i)
            {
                // fg[i + 1] = x[i + 1] - 2 * x[i] - x[i + 3]; 
            }

            return;
        }
     };
}

class Cost {
public:
    Cost(const std::vector<sFreeCircle> bubbles)
    {
        this->bubbles = bubbles;
        this->n_plan = bubbles.size();
    }
    ~Cost(){}

    /** @brief Basic scalar value type. */
    typedef AD<double> Scalar;

    /** @brief Differentiable variable ADvector type. */
    typedef CPPAD_TESTVECTOR(Scalar) ADvector;

    /** @brief Number of `(x, y)` points in the plan. */
    size_t n_plan;

    std::vector<sFreeCircle> bubbles;

    /**
     * @brief Compute the cost function for the OPP.
     */
    void operator () (ADvector &fg, const ADvector &vars) 
    {
        fg[0] = 0.0;   
        size_t i;

        for (i = 1; i < n_plan - 1; ++i) 
        {
            Scalar x_0 = vars[x_start + SIZEOF_POINT * (i - 1)];
            Scalar y_0 = vars[y_start + SIZEOF_POINT * (i - 1)];
            Scalar x_ = vars[x_start + SIZEOF_POINT * i];
            Scalar y_ = vars[y_start + SIZEOF_POINT * i];
            Scalar x_1 = vars[x_start + SIZEOF_POINT * (i + 1)];
            Scalar y_1 = vars[y_start + SIZEOF_POINT * (i + 1)];

            Scalar x_d0 = x_ - x_0;
            Scalar y_d0 = y_ - y_0;
            Scalar x_d1 = x_1 - x_;
            Scalar y_d1 = y_1 - y_;

            fg[0] += CppAD::pow(x_d1 - x_d0, 2);
            fg[0] += CppAD::pow(y_d1 - y_d0, 2);

            //Constraint functions values.
            fg[1 + CS0 + SIZEOF_CONSTRAINT * i] = x_ - bubbles.at(i).center.x;
            fg[1 + CS1 + SIZEOF_CONSTRAINT * i] = y_ - bubbles.at(i).center.y;
        }

        // i = 0;
        // fg[1 + CS0 + SIZEOF_CONSTRAINT * i] = vars[x_start + SIZEOF_POINT * i] - bubbles.at(i).center.x;
        // fg[1 + CS1 + SIZEOF_CONSTRAINT * i] = vars[y_start + SIZEOF_POINT * i] - bubbles.at(i).center.y;

        // i = n_plan - 1;
        // fg[1 + CS0 + SIZEOF_CONSTRAINT * i] = vars[x_start + SIZEOF_POINT * i] - bubbles.at(i).center.x;
        // fg[1 + CS1 + SIZEOF_CONSTRAINT * i] = vars[y_start + SIZEOF_POINT * i] - bubbles.at(i).center.y;

    }
};

//
// Optimizer class definition implementation.
//
Optimizer::Optimizer() {}
Optimizer::~Optimizer() {}

void Optimizer::GetStarted()
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR( double ) Dvector;

    // number of independent variables (domain dimension for f and g)
    size_t nx = 3;
    // number of constraints (range dimension for g)
    size_t ng = 2;
    // initial value of the independent variables
    nx += ng;
    Dvector xi(nx);
    xi[0] = 10.0;
    xi[1] = 10.0;
    xi[2] = 10.0;

    xi[3] = 1.0;
    xi[4] = 1.0;

    // lower and upper limits for x
    Dvector xl(nx), xu(nx);
    for(i = 0; i < nx; i++)
    {     
      xl[i] = -5.0;
      xu[i] = 5.0;
    }
    xl[0] = 3.0;
    xu[0] = 3.0;

    // lower and upper limits for g
    Dvector gl(ng), gu(ng);
    gl[0] = -10.0;     
    gu[0] = 10.0;
    gl[1] = -10.0;     
    gu[1] = 10.0;

    gl[0] = 0.0;     
    gu[0] = 0.0;
    gl[1] = 0.0;     
    gu[1] = 0.0;

    // object that computes objective and constraints
    FG_eval fg_eval;

    // options
    std::string options;
    // // turn off any printing
    // options += "Integer print_level  0\n";
    // options += "String  sb           yes\n";
    // // maximum number of iterations
    // options += "Integer max_iter     20\n";
    // // approximate accuracy in first order necessary conditions;
    // // see Mathematical Programming, Volume 106, Number 1,
    // // Pages 25-57, Equation (6)
    // options += "Numeric tol          1e-6\n";
    // // derivative testing
    // options += "String  derivative_test            second-order\n";
    // // maximum amount of random pertubation; e.g.,
    // // when evaluation finite diff
    // options += "Numeric point_perturbation_radius  0.\n";

    options += "Integer print_level 0\n";
    options += "Sparse true reverse\n";
    options += "Numeric max_cpu_time 0.2\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, 
        xi, 
        xl, 
        xu, 
        gl, 
        gu, 
        fg_eval, 
        solution
    );

    //
    // Check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    auto status = (ok ? "succeeded !!!\n" : "failed !!!\n");
    std::cout << "Solver " << status << "Final cost value = " << solution.obj_value << std::endl;

    for(i = 0; i < nx; ++i)
    {     
        std::cout << i << " ----solution.x[i]: " << solution.x[i] << std::endl;
        // std::cout << " ----solution.zl[i]: " << solution.zl[i] << std::endl;
        // std::cout << " ----solution.zu[i]: " << solution.zu[i] << std::endl;
    } 

    std::cout << "****    ****" << std::endl;

    for (i = 0; i < ng; ++i)
    {
        std::cout << i << " ----solution.g[i]: " << solution.g[i] << std::endl;
    }
    return;
}

bool Optimizer::Solve(std::vector<sFreeCircle> bubbles)
{
    bool ok = true;
    size_t i, j;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // number of independent variables (domain dimension for f and g)
    size_t nx = bubbles.size() * SIZEOF_POINT;

    // number of constraints (range dimension for g)
    size_t ng = bubbles.size() * SIZEOF_CONSTRAINT;
    // initial value of the independent variables
    Dvector xi(nx);
    // lower and upper limits for x
    Dvector xl(nx), xu(nx);
    // lower and upper limits for g
    Dvector gl(ng), gu(ng);

    j = 0;
    for (i = 0; i < nx; i += SIZEOF_POINT)
    {
        size_t i_x = i + x_start;
        size_t i_y = i + y_start;
        xi[i_x] = 0.0;
        xi[i_y] = 0.0;

        xl[i_x] = -1.0e19;
        xl[i_y] = -1.0e19;

        xu[i_x] = 1.0e19;
        xu[i_y] = 1.0e19;

        xl[i_x] = bubbles.at(j).center.x - bubbles.at(j).radius * 0.5;
        xl[i_y] = bubbles.at(j).center.y - bubbles.at(j).radius * 0.5;

        xu[i_x] = bubbles.at(j).center.x + bubbles.at(j).radius * 0.5;
        xu[i_y] = bubbles.at(j).center.y + bubbles.at(j).radius * 0.5;
        j++;
    }

    xl[0] = bubbles.front().center.x;
    xu[0] = bubbles.front().center.x;
    xl[1] = bubbles.front().center.y;
    xu[1] = bubbles.front().center.y;

    xl[2] = bubbles.at(1).center.x;
    xu[2] = bubbles.at(1).center.x;
    xl[3] = bubbles.at(1).center.y;
    xu[3] = bubbles.at(1).center.y;

    xl[nx - 2] = bubbles.back().center.x;
    xu[nx - 2] = bubbles.back().center.x;
    xl[nx - 1] = bubbles.back().center.y;
    xu[nx - 1] = bubbles.back().center.y;

    xl[nx - 4] = bubbles.at(bubbles.size() - 2).center.x;
    xu[nx - 4] = bubbles.at(bubbles.size() - 2).center.x;
    xl[nx - 3] = bubbles.at(bubbles.size() - 2).center.y;
    xu[nx - 3] = bubbles.at(bubbles.size() - 2).center.y;

    j = 0;
    for (i = 0; i < ng; i += SIZEOF_CONSTRAINT)
    {
        size_t i_x = i + x_start;
        size_t i_y = i + y_start;

        gl[i_x] = -bubbles.at(j).radius * 0.5;
        gu[i_x] = bubbles.at(j).radius * 0.5;

        gl[i_y] = -bubbles.at(j).radius * 0.5;
        gu[i_y] = bubbles.at(j).radius * 0.5;

        gl[i_x] = -1.0e19;
        gu[i_x] = 1.0e19;

        gl[i_y] = -1.0e19;
        gu[i_y] = 1.0e19;

        j++;
    }

    // gl[0] = 0.0;
    // gu[0] = 0.0;
    // gl[1] = 0.0;
    // gu[1] = 0.0;

    // gl[nx - 2] = 0.0;
    // gu[nx - 2] = 0.0;
    // gl[nx - 1] = 0.0;
    // gu[nx - 1] = 0.0;


    // object that computes objective and constraints
    Cost fg_eval(bubbles);

    // options
    std::string options;
    // // turn off any printing
    // options += "Integer print_level  0\n";
    // options += "String  sb           yes\n";
    // // maximum number of iterations
    // options += "Integer max_iter     20\n";
    // // approximate accuracy in first order necessary conditions;
    // // see Mathematical Programming, Volume 106, Number 1,
    // // Pages 25-57, Equation (6)
    // options += "Numeric tol          1e-6\n";
    // // derivative testing
    // options += "String  derivative_test            second-order\n";
    // // maximum amount of random pertubation; e.g.,
    // // when evaluation finite diff
    // options += "Numeric point_perturbation_radius  0.\n";

    options += "Integer print_level 0\n";
    options += "Sparse true reverse\n";
    options += "Numeric max_cpu_time 0.2\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, Cost>(
        options, 
        xi, 
        xl, 
        xu, 
        gl, 
        gu, 
        fg_eval, 
        solution
    );

    //
    // Check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    auto status = (ok ? "succeeded !!!\n" : "failed !!!\n");
    std::cout << "Solver " << status << "Final cost value = " << solution.obj_value << std::endl;
    std::cout << "solution.status: " << solution.status << std::endl;
    std::cout << "solution.x.size(): " << solution.x.size() << "; nx: " << nx << std::endl;

    geometry_msgs::Point32 temp;
    path.clear();
    for(i = 0; i < solution.x.size(); i += SIZEOF_POINT)
    {     
        size_t i_x = i + x_start;
        size_t i_y = i + y_start;
        temp.x = solution.x[i_x];
        temp.y = solution.x[i_y];
        path.push_back(temp);
    } 

    // for (i = 0; i < solution.x.size(); ++i)
    // {
    //     std::cout << "solution.x[i]: " << solution.x[i] << std::endl;
    // }

    // for (i = 0; i < solution.g.size(); ++i)
    // {
    //     std::cout << i << " ----solution.g[i]: " << solution.g[i] << std::endl;
    // }

    return ok;

}
