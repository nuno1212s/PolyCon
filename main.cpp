#include <iostream>
#include <vector>
#include "polygon_construction.h"
#include "dcel/dcel.h"

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cout << "Please provide the N and the M arguments, in that order as an argument" << std::endl;

        return EXIT_FAILURE;
    }

    int N = atoi(argv[1]),
            M = atoi(argv[2]);

    //int N = 25, M = 15;

    std::cout << "Running for N: " << N << " M: " << M << std::endl;

    generateIntNPointsWithinM(N, M);

    std::cout << "Result exported to result.json, use the visualize.py to see the polygon." << std::endl;

    return EXIT_SUCCESS;
}