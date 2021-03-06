#ifndef ARTERY_GTUINETMOBILITY_H_T7FXXEAE
#define ARTERY_GTUINETMOBILITY_H_T7FXXEAE

#include "artery/utility/Geometry.h"
#include "ots/GtuSink.h"
#include <inet/mobility/contract/IMobility.h>
#include <omnetpp/csimplemodule.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <etsi_its_msgs/msg/cam.hpp>

#include <ots/RosNode.h>

namespace inet {
    class CanvasProjection;
    class IGeographicCoordinateSystem;
} // inet

namespace artery
{

class GtuInetMobility :
    public inet::IMobility, public ots::GtuSink,
    public omnetpp::cSimpleModule
{
public:
    GtuInetMobility();
    virtual ~GtuInetMobility();

    void initialize(int stage) override;
    int numInitStages() const override;

    // inet::IMobility interface
    double getMaxSpeed() const override;
    inet::Coord getCurrentPosition() override;
    inet::Coord getCurrentSpeed() override;
    inet::EulerAngles getCurrentAngularPosition() override;
    inet::EulerAngles getCurrentAngularSpeed() override;
    inet::Coord getConstraintAreaMax() const override;
    inet::Coord getConstraintAreaMin() const override;

    // ots::GtuSink
    void initialize(const ots::GtuObject&) override;
    void update(const ots::GtuObject&) override;

    const ots::GtuObject& getLastGtuObject() const { return mLastGtuObject; }
    Position getPosition() const;
    GeoPosition getGeoPosition() const;
    Angle getHeading() const;

protected:
    virtual void setInetProperties(const ots::GtuObject&);
    virtual void updateVisualRepresentation();

private:
    ots::GtuObject mLastGtuObject;
    inet::Coord mPosition;
    inet::Coord mSpeed;
    inet::EulerAngles mOrientation;
    omnetpp::cModule* mVisualRepresentation = nullptr;
    const inet::CanvasProjection* mCanvasProjection = nullptr;
    const inet::IGeographicCoordinateSystem* mCoordinateSystem = nullptr;

    RosNode rosNode;
    rclcpp::Subscription<etsi_its_msgs::msg::CAM>::SharedPtr camSub;
    void cam_callback(const etsi_its_msgs::msg::CAM::SharedPtr msg);

};

} // namespace artery

#endif /* ARTERY_GTUINETMOBILITY_H_T7FXXEAE */
