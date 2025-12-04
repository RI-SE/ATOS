# RESTBridge

## About the module
RESTBridge is a module that allows you to send HTTP requests to a REST API through ATOS. The module utilize OpenScenario CustomCommandActions to specify the type of request (Currently POST and DELETE are supported) and the content of the request.

## Example OpenScenario

```xml
<Event name="web_event" priority="parallel" maximumExecutionCount="1">
                            <Action name="Send web request">
                                <UserDefinedAction>
                                    <CustomCommandAction type="POST">{"endpoint": "http://web_server/set_some_value", "data": {"some_key": "some_value"}}</CustomCommandAction>
                                </UserDefinedAction>
                            </Action>
                            <StartTrigger>
                                <ConditionGroup>
                                    <Condition name="Send web request" delay="0.0" conditionEdge="none">
                                        <ByEntityCondition>
                                            <TriggeringEntities triggeringEntitiesRule="any">
                                                <EntityRef entityRef="entity_name"/>
                                            </TriggeringEntities>
                                            <EntityCondition>
                                                <ReachPositionCondition tolerance="1.0">
                                                    <Position>
                                                        <LanePosition roadId="5" laneId="-1" s="45.0" offset="0.0"/>
                                                    </Position>
                                                </ReachPositionCondition>
                                            </EntityCondition>
                                        </ByEntityCondition>
                                    </Condition>
                                </ConditionGroup>
                            </StartTrigger>
                        </Event>
```

###  CustomCommandAction

type: POST or DELETE
data field:

```json
{
    "endpoint": "http://web_server/set_some_value", # Required. The endpoint to send the request to.
    "data": {"some_key": "some_value"} # Optional. The data to send in the request.
}
```

