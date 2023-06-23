#include <iostream>
#include <chrono>
#include <list>
#include <fstream>
#include <iterator>
#include <algorithm>

#include "smoothTrajectory.h"


typedef std::chrono::high_resolution_clock Clock;


template <typename Container>
void writeContainerToFile(const Container& container, const std::string& filename) {
    std::ofstream outputFile(filename);
    if (!outputFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::ostream_iterator<double> outputIterator(outputFile, "\n");
    std::copy(container.begin(), container.end(), outputIterator);

    outputFile.close();
    std::cout << "Data written to file: " << filename << std::endl;
}

int main()
{
    std::cout << "Testing smooth trajectory library" << std::endl;

    const double tStart{ 0.0 };
    const double tEnd{ 10.0 };
    const double pStart{ 0.0 };
    const double pEnd{ 2.0 };
    
    // Duration of the loop
    const std::chrono::duration<long, std::nano> tDuration{ static_cast<long>((tEnd-tStart)*1e9) };

    // Sampling time of the loop
    const std::chrono::duration<long, std::nano> h{ 100000000 };

    // Elapsed time from the beginning of the loop
    std::chrono::duration<long, std::nano> tElapsed{ 0 };

    SmoothTrajectory tr(tStart, tEnd, pStart, pEnd);

    // Calculated trajectory and time
    std::list<double> xd;
    std::list<double> xpd;
    std::list<double> time;
    
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

        xd.push_back(tr.jointTrajectoryPos(tElapsed.count()*1e-9));
        xpd.push_back(tr.jointTrajectoryVel(tElapsed.count()*1e-9));
        time.push_back(tElapsed.count()*1e-9);

        // Hold the iteratation for hFastLoop seconds in total
        while (t - tIterationStart < h)
            t = Clock::now();

        // Update the start time of the previous iteration of the loop
        tPrevIterationStart = tIterationStart;
    }

    // Save data to file
    writeContainerToFile(time, "time.csv");
    writeContainerToFile(xd, "desiredPosition.csv");

    return 0;
}