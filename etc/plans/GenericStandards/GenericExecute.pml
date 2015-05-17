<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1431522123418" name="GenericExecute" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GenericStandards" priority="0.0" minCardinality="2" maxCardinality="2147483647">
  <states id="1431522155979" name="GrabBall" comment="" entryPoint="1431522155980">
    <outTransitions>#1431522782044</outTransitions>
  </states>
  <states id="1431522297705" name="Align" comment="" entryPoint="1431522269326">
    <outTransitions>#1431522920716</outTransitions>
  </states>
  <states id="1431522763494" name="Pass" comment="">
    <inTransitions>#1431522782044</inTransitions>
    <outTransitions>#1431524869870</outTransitions>
  </states>
  <states id="1431522912251" name="Receive" comment="">
    <inTransitions>#1431522920716</inTransitions>
    <outTransitions>#1431523011459</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1431522995646" name="Success" comment="">
    <inTransitions>#1431523011459</inTransitions>
  </states>
  <states id="1431523482646" name="Block" comment="" entryPoint="1431523395534"/>
  <states id="1431524014799" name="Defend" comment="" entryPoint="1431523422152"/>
  <states id="1431524769489" name="SpatialDefend" comment="">
    <inTransitions>#1431524869870</inTransitions>
  </states>
  <transitions id="1431522782044" name="MISSING_NAME" comment="grab ball success and situation start" msg="">
    <preCondition id="1431522783626" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1431522155979</inState>
    <outState>#1431522763494</outState>
  </transitions>
  <transitions id="1431522920716" name="MISSING_NAME" comment="aligned" msg="">
    <preCondition id="1431522922124" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1431522297705</inState>
    <outState>#1431522912251</outState>
  </transitions>
  <transitions id="1431523011459" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1431523013533" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1431522912251</inState>
    <outState>#1431522995646</outState>
  </transitions>
  <transitions id="1431524869870" name="MISSING_NAME" comment="executed" msg="">
    <preCondition id="1431524871023" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1431522763494</inState>
    <outState>#1431524769489</outState>
  </transitions>
  <entryPoints id="1431522155980" name="MISSING_NAME" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1431522228653</task>
    <state>#1431522155979</state>
  </entryPoints>
  <entryPoints id="1431522269326" name="MISSING_NAME" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1431522275287</task>
    <state>#1431522297705</state>
  </entryPoints>
  <entryPoints id="1431523395534" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1431523409834</task>
    <state>#1431523482646</state>
  </entryPoints>
  <entryPoints id="1431523422152" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1431523425270</task>
    <state>#1431524014799</state>
  </entryPoints>
</alica:Plan>