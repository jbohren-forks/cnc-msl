#include "Plans/GenericStandards/GenericOppStandards1432132075122.h"
using namespace alica;
/*PROTECTED REGION ID(eph1432132075122) ENABLED START*/ //Add additional using directives here
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:GenericOppStandards

    /* generated comment
     
     Task: Blocker  -> EntryPoint-ID: 1432132075124

     Task: Keeper  -> EntryPoint-ID: 1432132127000

     */
    shared_ptr<UtilityFunction> UtilityFunction1432132075122::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1432132075122) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: Dummy in Plan: GenericOppStandards

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Sitaution= OppThrowin 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132367859::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132365582) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Sitaution= OppKickOff 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132369717::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132368065) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Sitaution= OppGoalKick 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132371055::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132369863) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Sitaution= OppPenaltyKick 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132372365::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132371135) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Sitaution= OppCornerKick 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132373471::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132372448) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : Sitaution= OppFreeKick 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132374502::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132373553) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: KeepGoal in Plan: GenericOppStandards

    //State: OppThrowIn in Plan: GenericOppStandards

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : any child success 
     *
     * Plans in State: 				
     *   - Plan - (Name): GenericDefend, (PlanID): 1432133473779 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132432036::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132429602) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: OppKickOff in Plan: GenericOppStandards

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : any child success 
     *
     * Plans in State: 				
     *   - Plan - (Name): GenericDefend, (PlanID): 1432133473779 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132434051::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132432185) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: OppGoalKick in Plan: GenericOppStandards

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : any child success 
     *
     * Plans in State: 				
     *   - Plan - (Name): GenericDefend, (PlanID): 1432133473779 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132436308::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132434174) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: OppPenaltyKick in Plan: GenericOppStandards

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : any child success 
     *
     * Plans in State: 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132438181::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132436499) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: OppFreeKick in Plan: GenericOppStandards

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : any child success 
     *
     * Plans in State: 				
     *   - Plan - (Name): GenericDefend, (PlanID): 1432133473779 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132442910::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132441185) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: OppCornerKick in Plan: GenericOppStandards

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment : any child success 
     *
     * Plans in State: 				
     *   - Plan - (Name): GenericDefend, (PlanID): 1432133473779 
     *
     * Tasks: 
     *   - Blocker (1225112227903) (Entrypoint: 1432132075124)
     *   - Keeper (1432132131330) (Entrypoint: 1432132127000)
     *
     * States:
     *   - Dummy (1432132075123)
     *   - KeepGoal (1432132173301)
     *   - OppThrowIn (1432132225844)
     *   - OppKickOff (1432132283325)
     *   - OppGoalKick (1432132299937)
     *   - OppPenaltyKick (1432132312429)
     *   - OppFreeKick (1432132325758)
     *   - OppCornerKick (1432132336430)
     *   - Success (1432132398157)
     *
     * Vars:
     */
    bool TransitionCondition1432132441034::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1432132438388) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

//State: Success in Plan: GenericOppStandards

}
