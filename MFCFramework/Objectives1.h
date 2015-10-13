#ifndef _OBJECTIVEFUNCTIONS_H
#define _OBJECTIVEFUNCTIONS_H

#include <fstream>
#include <shark/ObjectiveFunctions/AbstractObjectiveFunction.h>
#include <shark/Rng/GlobalRng.h>

#include "ControllerFramwork.h"

#include <time.h>
#include <windows.h>
#include <conio.h>
#include <iostream>

using namespace std;

namespace shark {
    /*! \brief Multi-objective optimization benchmark function ZDT1
    *
    *  The function is described in
    *
    *  Eckart Zitzler, Kalyanmoy Deb, and Lothar Thiele. Comparison of
    *  Multiobjective Evolutionary Algorithms: Empirical
    *  Results. Evolutionary Computation 8(2):173-195, 2000
    */

    class ObjectiveFunctions : public MultiObjectiveFunction
    {
    public:
        ObjectiveFunctions(std::size_t dimensions = 27, std::size_t objectives = 2)
            :m_dimensions(dimensions), m_objectives(objectives)
        {
            m_features |= CAN_PROPOSE_STARTING_POINT;
        }

        std::string name() 
        {
            return "ObjectiveFunctions";
        }

        std::size_t numberOfVariables()const
        {
            return m_dimensions;
        }

        std::size_t numberOfObjectives()const
        {
            return m_objectives;
        }

        void proposeStartingPoint( SearchPointType & startingPoint )const {
            ifstream file;
            file.open("../Data/Optimization/fwalk.in");
            double p;
            startingPoint.resize(m_dimensions);
            for(std::size_t i = 0; i != m_dimensions; ++i){
                file >> p;
                startingPoint(i) = p + Rng::gauss(0, 0.0025);
            }
            file.close();
        }

        ResultType eval( const SearchPointType & input )const 
        {
            m_evaluationCounter++;
            _cprintf("%d\n", m_evaluationCounter);
//             if(m_evaluationCounter < 200)
//             {
//                 ResultType result(m_objectives);
//                 result(0) = 0;
//                 result(1) = 0;
//                 return result;
//             }

            vector<double> tmpp;
            _cprintf("input:\n");
            for( std::size_t i = 0; i < m_dimensions; ++i ) {
                _cprintf("%lf ", input(i));
                tmpp.push_back(input(i));
            }
            (Globals::app)->reloadParameters(tmpp);
            _cprintf("\n\n");
            int controlShotToWrite = 0;
            Vector3d footSize = (Globals::app)->TenStep(controlShotToWrite);
            _cprintf("output: %lf %lf %lf\n\n", footSize.x, footSize.y, footSize.z);
            ResultType result(m_objectives);
            result(0) = abs(footSize.x*1000 - 300);
            result(1) = abs(footSize.z*1000 - 100);
            if(m_evaluationCounter >= 5000 && m_evaluationCounter <= 5009)
            {
                char stateFileName[100]="";
                char fileName[100]="";

                sprintf(stateFileName, "../Data/controlShots/controlshots%05d.rs", controlShotToWrite);
                (Globals::app)->conF->getCharacter()->saveReducedStateToFile(stateFileName);                
                
                sprintf(fileName, "../Data/controlShots/controlshots%05d.sbc", controlShotToWrite);
                ofstream resFile;
                resFile.open(fileName);
                for(size_t j = 0; j < 27; ++j)
                {
                    resFile << input[j] << std::endl;		
                }
                resFile << footSize.x << " " << footSize.z << endl;
                resFile.close();
            }

            _cprintf("res: %lf %lf\n\n", result(0), result(1));
            
            //more objectives here...
            //system("pause");
            return result;
        }
    private:
        double m_a;
        std::size_t m_dimensions;
        std::size_t m_objectives;
    };

}
#endif