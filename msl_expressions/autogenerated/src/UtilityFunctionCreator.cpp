#include <iostream>
#include "UtilityFunctionCreator.h"

#include  "Plans/WM161413992564408.h"

#include  "Plans/TwoHoledWall/TwoHoledWallMaster1417621468963.h"

#include  "Plans/ActuatorTest/ActuatorTestMaster1417017436952.h"

#include  "Plans/TwoHoledWall/ShootTwoHoledWall1417620189234.h"

using namespace std;
using namespace alicaAutogenerated;
namespace alica
{

    UtilityFunctionCreator::~UtilityFunctionCreator()
    {
    }

    UtilityFunctionCreator::UtilityFunctionCreator()
    {
    }

    shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(long utilityfunctionConfId)
    {
        switch (utilityfunctionConfId)
        {

            case 1413992564408:
                return make_shared<UtilityFunction1413992564408>();
                break;

            case 1417621468963:
                return make_shared<UtilityFunction1417621468963>();
                break;

            case 1417017436952:
                return make_shared<UtilityFunction1417017436952>();
                break;

            case 1417620189234:
                return make_shared<UtilityFunction1417620189234>();
                break;

            default:
                cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << endl;
                throw new exception();
                break;
        }
    }

}
