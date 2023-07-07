#include <iostream>
#include <chrono>
#include <list>
#include <fstream>
#include <iterator>
#include <algorithm>

#include "smoothTrajectory.h"
#include "Eigen/Dense"


typedef std::chrono::high_resolution_clock Clock;


template <typename Container>
void writeContainerToFile(const Container& container, const std::string& filename) {
    std::ofstream outputFile(filename);
    if (!outputFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::ostream_iterator<Eigen::Vector3d> outputIterator(outputFile, "\n");
    std::copy(container.begin(), container.end(), outputIterator);

    outputFile.close();
    std::cout << "Data written to file: " << filename << std::endl;
}

int main()
{
    std::cout << "Testing smooth trajectory library" << std::endl;

    const float tStart{ 0.0f };
    const float tEnd{ 10.0f };
    const Eigen::Vector3f pStart{ 0.0, 0.0, 0.0 };
    const Eigen::Vector3f pEnd{ 1.0, 2.0, 3.0 };
    
    // Duration of the loop
    const std::chrono::duration<long long, std::nano> tDuration{ static_cast<long long>((tEnd-tStart)*1e9) };

    // Sampling time of the loop
    const std::chrono::duration<long long, std::nano> h{ 100000000 };

    // Elapsed time from the beginning of the loop
    std::chrono::duration<long long, std::nano> tElapsed{ 0 };

    SmoothTrajectory<float> tr(tStart, tEnd, pStart, pEnd);

    // Calculated trajectory and time
    std::list<Eigen::Vector3f> xd;
    std::list<Eigen::Vector3f> xpd;
    std::list<float> time;
    
    // Current timestamp
    auto t = Clock::now();

    // Start time of the loop
    const auto tLoopStart = t;

    // Start time of an iteration of the loop
    auto tIterationStart = t;

    // Start time of the previous iteration of the loop
    auto tPrevIterationStart = t;

    // Real-time loop
    while (tElapsed < tDuration)
    {
        // Update the start time of the iteration
        tIterationStart = Clock::now();

        // Calculate the elapsed time from the loop start
        tElapsed = tIterationStart-tLoopStart;

        xd.push_back(tr.jointTrajectoryPos(tElapsed.count()*1e-9f));
        xpd.push_back(tr.jointTrajectoryVel(tElapsed.count()*1e-9f));
        time.push_back(tElapsed.count()*1e-9f);

        // Hold the iteratation for hFastLoop seconds in total
        while (t - tIterationStart < h)
            t = Clock::now();

        // Update the start time of the previous iteration of the loop
        tPrevIterationStart = tIterationStart;
    }

    // Save data to file
    //writeContainerToFile(time, "time.csv");
    //writeContainerToFile(xd, "desiredPosition.csv");

    return 0;
}