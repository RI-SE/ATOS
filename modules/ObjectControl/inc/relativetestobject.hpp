#include "testobject.hpp"

class RelativeTestObject : public TestObject {
public:
	RelativeTestObject(uint32_t id, uint32_t anchorID);
	RelativeTestObject(const RelativeTestObject&) = delete;
	RelativeTestObject(RelativeTestObject&&);

	RelativeTestObject& operator=(const RelativeTestObject&) = delete;
	RelativeTestObject& operator=(RelativeTestObject&&) = default;

private:
    uint32_t anchor;
    bool isAnchor;
    ObjectMonitorType transformCoordinate(const ObjectMonitorType& point,
        const ObjectMonitorType& anchor,
        const bool debug);
    MonitorMessage readMonitorMessage() override;
};
