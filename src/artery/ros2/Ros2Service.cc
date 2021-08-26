//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "artery/ros2/Ros2Service.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cpacket.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>

#include <ots/GtuObject.h>
#include "artery/utility/IdentityRegistry.h"
#include "artery/envmod/sensor/CamSensor.h"

using namespace omnetpp;
using namespace vanetza;

namespace artery
{

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

Define_Module(Ros2Service)

Ros2Service::Ros2Service()
{
}

Ros2Service::~Ros2Service()
{
	cancelAndDelete(m_self_msg);
}

void Ros2Service::indicate(const btp::DataIndication& ind, cPacket* packet, const NetworkInterface& net)
{
	Enter_Method("indicate");

	if (packet->getByteLength() == 42) {
		EV_INFO << "packet indication on channel " << net.channel << "\n";
	}

	delete(packet);
}

void Ros2Service::initialize()
{
	ItsG5Service::initialize();
	m_self_msg = new cMessage("Example Service");
	subscribe(scSignalCamReceived);

	scheduleAt(simTime() + 3.0, m_self_msg);
}

void Ros2Service::finish()
{
	// you could record some scalars at this point
	ItsG5Service::finish();
}

void Ros2Service::handleMessage(cMessage* msg)
{
	Enter_Method("handleMessage");

	if (msg == m_self_msg) {
		EV_INFO << "self message\n";
	}
}

void Ros2Service::trigger()
{
	Enter_Method("trigger");

	// use an ITS-AID reserved for testing purposes
	static const vanetza::ItsAid example_its_aid = 16480;

	auto& mco = getFacilities().get_const<MultiChannelPolicy>();
	auto& networks = getFacilities().get_const<NetworkInterfaceTable>();

	for (auto channel : mco.allChannels(example_its_aid)) {
		auto network = networks.select(channel);
		if (network) {
			btp::DataRequestB req;
			// use same port number as configured for listening on this channel
			req.destination_port = host_cast(getPortNumber(channel));
			req.gn.transport_type = geonet::TransportType::SHB;
			req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
			req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
			req.gn.its_aid = example_its_aid;

			cPacket* packet = new cPacket("Example Service Packet");
			packet->setByteLength(42);

			// send packet on specific network interface
			request(req, packet, network.get());
		} else {
			EV_ERROR << "No network interface available for channel " << channel << "\n";
		}
	}
}

void Ros2Service::receiveSignal(cComponent* source, simsignal_t signal, cObject* object, cObject* details)
{
	Enter_Method("receiveSignal");

	std::cout << "service received cam" << std::endl;

    auto& gtu = getFacilities().getConst<ots::GtuObject>();
    std::string mGtuId = gtu.getId();

	std::cout << mGtuId << "received cam" << std::endl;

	if (auto cam = dynamic_cast<CaObject*>(object)) {
		const auto id = cam->asn1()->header.stationID;
		std::cout << id << std::endl;
	}

	if (auto cam = dynamic_cast<CaObject*>(object)) {
		const auto id = cam->asn1()->header.stationID;
		std::cout << "latitude " << cam->asn1()->cam.camParameters.basicContainer.referencePosition.latitude << std::endl;
		std::cout << "longitude " << cam->asn1()->cam.camParameters.basicContainer.referencePosition.longitude << std::endl;
		std::cout << "altitude " << cam->asn1()->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue << std::endl;
		std::cout << "stationtype " << cam->asn1()->cam.camParameters.basicContainer.stationType << std::endl;
		std::cout << "speed " << cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue << std::endl;
		std::cout << "acceleration " << cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue << std::endl;
		std::cout << "acceleration lateral " << cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration << std::endl;
		std::cout << "acceleration vertical " << cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.verticalAcceleration << std::endl;
		std::cout << "heading " << cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue << std::endl;
		std::cout << "yawRate " << cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue << std::endl;
		std::cout << "steeringWheelAngle " << cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.steeringWheelAngle << std::endl;
		std::cout << "messageId " << cam->asn1()->header.messageID << std::endl;
		std::cout << "stationid " << id << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
	}
}

} // namespace artery
