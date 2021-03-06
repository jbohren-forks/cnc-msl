#include "Plans/GenericStandards/GenericExecute1431522123418.h"
using namespace alica;
/*PROTECTED REGION ID(eph1431522123418) ENABLED START*/ //Add additional using directives here
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:GenericExecute

    /* generated comment
     
     Task: StandardExecuter  -> EntryPoint-ID: 1431522155980

     Task: StandardReceiver  -> EntryPoint-ID: 1431522269326

     Task: Blocker  -> EntryPoint-ID: 1431523395534

     Task: Defend  -> EntryPoint-ID: 1431523422152

     */
    shared_ptr<UtilityFunction> UtilityFunction1431522123418::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1431522123418) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: GrabBall in Plan: GenericExecute

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : grab ball success and situation start 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - StandardExecuter (1238601692867) (Entrypoint: 1431522155980)
     *   - StandardReceiver (1238601842183) (Entrypoint: 1431522269326)
     *   - Blocker (1432209050494) (Entrypoint: 1431523395534)
     *   - Defend (1225115406909) (Entrypoint: 1431523422152)
     *
     * States:
     *   - GrabBall (1431522155979)
     *   - Align (1431522297705)
     *   - Pass (1431522763494)
     *   - Receive (1431522912251)
     *   - Success (1431522995646)
     *   - Block (1431523482646)
     *   - Defend (1431524014799)
     *   - SpatialDefend (1431524769489)
     *
     * Vars:
     */
    bool TransitionCondition1431522783626::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1431522782044) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: Align in Plan: GenericExecute

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : aligned 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - StandardExecuter (1238601692867) (Entrypoint: 1431522155980)
     *   - StandardReceiver (1238601842183) (Entrypoint: 1431522269326)
     *   - Blocker (1432209050494) (Entrypoint: 1431523395534)
     *   - Defend (1225115406909) (Entrypoint: 1431523422152)
     *
     * States:
     *   - GrabBall (1431522155979)
     *   - Align (1431522297705)
     *   - Pass (1431522763494)
     *   - Receive (1431522912251)
     *   - Success (1431522995646)
     *   - Block (1431523482646)
     *   - Defend (1431524014799)
     *   - SpatialDefend (1431524769489)
     *
     * Vars:
     */
    bool TransitionCondition1431522922124::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1431522920716) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: Pass in Plan: GenericExecute

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : executed 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - StandardExecuter (1238601692867) (Entrypoint: 1431522155980)
     *   - StandardReceiver (1238601842183) (Entrypoint: 1431522269326)
     *   - Blocker (1432209050494) (Entrypoint: 1431523395534)
     *   - Defend (1225115406909) (Entrypoint: 1431523422152)
     *
     * States:
     *   - GrabBall (1431522155979)
     *   - Align (1431522297705)
     *   - Pass (1431522763494)
     *   - Receive (1431522912251)
     *   - Success (1431522995646)
     *   - Block (1431523482646)
     *   - Defend (1431524014799)
     *   - SpatialDefend (1431524769489)
     *
     * Vars:
     */
    bool TransitionCondition1431524871023::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1431524869870) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: Receive in Plan: GenericExecute

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : success 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - StandardExecuter (1238601692867) (Entrypoint: 1431522155980)
     *   - StandardReceiver (1238601842183) (Entrypoint: 1431522269326)
     *   - Blocker (1432209050494) (Entrypoint: 1431523395534)
     *   - Defend (1225115406909) (Entrypoint: 1431523422152)
     *
     * States:
     *   - GrabBall (1431522155979)
     *   - Align (1431522297705)
     *   - Pass (1431522763494)
     *   - Receive (1431522912251)
     *   - Success (1431522995646)
     *   - Block (1431523482646)
     *   - Defend (1431524014799)
     *   - SpatialDefend (1431524769489)
     *
     * Vars:
     */
    bool TransitionCondition1431523013533::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1431523011459) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

//State: Success in Plan: GenericExecute

//State: Block in Plan: GenericExecute

//State: Defend in Plan: GenericExecute

//State: SpatialDefend in Plan: GenericExecute

}
