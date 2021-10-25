// MIT License
// -----------
//
// Copyright (c) 2021 University of Malaga
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Author: Gonzalo J. Paz Delgado
// Supervisor: Carlos J. Pérez del Pulgar
// Affiliation: Department of Systems Engineering and Automation
// Space Robotics Lab (www.uma.es/space-robotics)

#ifndef __FAST_MARCHING__
#define __FAST_MARCHING__

#include <vector>

#define nocolor "\033[0m"
#define black "\033[1;30m"
#define red "\033[1;31m"
#define green "\033[1;32m"
#define yellow "\033[1;33m"
#define blue "\033[1;34m"
#define magenta "\033[1;35m"
#define cyan "\033[1;36m"
#define white "\033[1;37m"

namespace FastMarching
{
class PathPlanner
{
private:
    // -- PARAMETERS --
    double waypointDistance;

    void computeEntireTMap(const std::vector<std::vector<double>> * costMap,
                           std::vector<int> goal,
                           std::vector<std::vector<double>> * closedMap,
                           std::vector<std::vector<double>> * TMap);

    bool computeTMap(const std::vector<std::vector<double>> * costMap,
                     std::vector<int> goal,
                     std::vector<int> start,
                     std::vector<std::vector<double>> * TMap);

    void updateNode(std::vector<int> nodeTarget,
                    const std::vector<std::vector<double>> * costMap,
                    std::vector<std::vector<double>> * TMap,
                    std::vector<double> * nbT,
                    std::vector<std::vector<int>> * nbNodes,
                    const std::vector<std::vector<double>> * closedMap);

    double getEikonal(double THor, double TVer, double cost);

    int getInsertIndex(std::vector<double> * nbT, double T);

    void computePathGDM(const std::vector<std::vector<double>> * TMap,
                        std::vector<int> initNode,
                        std::vector<int> endNode,
                        double tau,
                        std::vector<std::vector<double>> * path);

    void computeGradient(const std::vector<std::vector<double>> * TMap,
                         std::vector<double> point,
                         std::vector<std::vector<double>> * Gnx,
                         std::vector<std::vector<double>> * Gny);

    double getInterpolatedPoint(std::vector<double> point,
                                const std::vector<std::vector<double>> * mapI);

public:
    // -- FUNCTIONS --
    PathPlanner(double _waypointDistance = 0.5);
    ~PathPlanner();

    bool planPath(const std::vector<std::vector<double>> * costMap,
                  double mapResolution,
                  std::vector<double> iniPos,
                  std::vector<double> finalPos,
                  std::vector<std::vector<double>> * path);
};
}    // namespace FastMarching
#endif
