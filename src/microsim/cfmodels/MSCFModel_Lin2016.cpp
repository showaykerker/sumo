/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2021 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSCFModel_Lin2016.cpp
/// @author  Hsu Hsiu Wei
/// @date    Oct 2021
///
// Car-following model based on [TBD].
/****************************************************************************/
#include <config.h>

#include <stdio.h>
#include <list>
#include <iostream>

#include "MSCFModel_Lin2016.h"
#include <microsim/MSNet.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSRoute.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOTime.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <math.h>

// ===========================================================================
// debug flags
// ===========================================================================
//#define DEBUG_LIN2016
#define DEBUG_COND (true)
#define DEBUG_GET_INVOLVED
// #define DEBUG_COND (veh->isSelected())

typedef std::vector<MSEdge*> MSEdgeVector;
typedef std::vector<std::pair<double, const SUMOVehicle*>> MSVehIDInstanceVector;

// ===========================================================================
// constants
// ===========================================================================
// #define PI 3.14159265  // already defined in /usr/include/fox-1.6/fxdefs.h

// ===========================================================================
// defaults
// ===========================================================================
#define DEFAULT_SC_GAIN -0.4
#define DEFAULT_GCC_GAIN_SPEED 0.8
#define DEFAULT_GCC_GAIN_SPACE 0.04
#define DEFAULT_GC_GAIN_SPEED 0.07
#define DEFAULT_GC_GAIN_SPACE 0.23
#define DEFAULT_CA_GAIN_SPACE 0.8
#define DEFAULT_CA_GAIN_SPEED 0.23

#define DEFAULT_LOOKAHEAD 50.0
// ===========================================================================
// thresholds
// ===========================================================================
#define GAP_THRESHOLD_SPEEDCTRL 120
#define GAP_THRESHOLD_GAPCTRL 100




// override followSpeed when deemed unsafe by the given margin (the value was selected to reduce the number of necessary interventions)
#define DEFAULT_EMERGENCY_OVERRIDE_THRESHOLD 2.0

/// @todo: add attributes for myCollisionAvoidanceGainSpeed and myCollisionAvoidanceGainSpace

// ===========================================================================
// method definitions
// ===========================================================================
MSCFModel_Lin2016::MSCFModel_Lin2016(const MSVehicleType* vtype) :
    MSCFModel(vtype),
    myLookaheadDist(vtype->getParameter().getCFParam(SUMO_ATTR_CF_L16_LOOKAHEAD, DEFAULT_LOOKAHEAD)),
    mySpeedControlGain(vtype->getParameter().getCFParam(SUMO_ATTR_SC_GAIN, DEFAULT_SC_GAIN)),
    myGapClosingControlGainSpeed(vtype->getParameter().getCFParam(SUMO_ATTR_GCC_GAIN_SPEED, DEFAULT_GCC_GAIN_SPEED)),
    myGapClosingControlGainSpace(vtype->getParameter().getCFParam(SUMO_ATTR_GCC_GAIN_SPACE, DEFAULT_GCC_GAIN_SPACE)),
    myGapControlGainSpeed(vtype->getParameter().getCFParam(SUMO_ATTR_GC_GAIN_SPEED, DEFAULT_GC_GAIN_SPEED)),
    myGapControlGainSpace(vtype->getParameter().getCFParam(SUMO_ATTR_GC_GAIN_SPACE, DEFAULT_GC_GAIN_SPACE)),
    myCollisionAvoidanceGainSpeed(vtype->getParameter().getCFParam(SUMO_ATTR_CA_GAIN_SPEED, DEFAULT_CA_GAIN_SPEED)),
    myCollisionAvoidanceGainSpace(vtype->getParameter().getCFParam(SUMO_ATTR_CA_GAIN_SPACE, DEFAULT_CA_GAIN_SPACE)) {
    // ACC does not drive very precise and often violates minGap
    myCollisionMinGapFactor = vtype->getParameter().getCFParam(SUMO_ATTR_COLLISION_MINGAP_FACTOR, 0.1);
}

MSCFModel_Lin2016::~MSCFModel_Lin2016() {}


double
MSCFModel_Lin2016::followSpeed(const MSVehicle* const veh, double speed, double gap2pred, double predSpeed, double predMaxDecel, const MSVehicle* const /*pred*/) const {
    const double desSpeed = MIN2(veh->getLane()->getSpeedLimit(), veh->getMaxSpeed());
    const double vACC = _v(veh, gap2pred, speed, predSpeed, desSpeed, true);
    const double vSafe = maximumSafeFollowSpeed(gap2pred, speed, predSpeed, predMaxDecel);
    if (vSafe + DEFAULT_EMERGENCY_OVERRIDE_THRESHOLD < vACC) {
        //Lin2016VehicleVariables* vars = (Lin2016VehicleVariables*)veh->getCarFollowVariables();
        //std::cout << SIMTIME << " veh=" << veh->getID() << " v=" << speed << " vL=" << predSpeed << " gap=" << gap2pred << " vACC=" << vACC << " vSafe=" << vSafe << " cm=" << vars->ACC_ControlMode << "\n";
        return vSafe + DEFAULT_EMERGENCY_OVERRIDE_THRESHOLD;
    }
    return vACC;
}


double
MSCFModel_Lin2016::stopSpeed(const MSVehicle* const veh, const double speed, double gap, double decel) const {
    // NOTE: This allows return of smaller values than minNextSpeed().
    // Only relevant for the ballistic update: We give the argument headway=TS, to assure that
    // the stopping position is approached with a uniform deceleration also for tau!=TS.
    return MIN2(maximumSafeStopSpeed(gap, decel, speed, false, veh->getActionStepLengthSecs()), maxNextSpeed(speed, veh));
}


double
MSCFModel_Lin2016::getSecureGap(const MSVehicle* const /*veh*/, const MSVehicle* const /*pred*/, const double speed, const double leaderSpeed, const double /* leaderMaxDecel */) const {
    // Accel in gap mode should vanish:
    //      0 = myGapControlGainSpeed * (leaderSpeed - speed) + myGapControlGainSpace * (g - myHeadwayTime * speed);
    // <=>  myGapControlGainSpace * g = - myGapControlGainSpeed * (leaderSpeed - speed) + myGapControlGainSpace * myHeadwayTime * speed;
    // <=>  g = - myGapControlGainSpeed * (leaderSpeed - speed) / myGapControlGainSpace + myHeadwayTime * speed;
    return myGapControlGainSpeed * (speed - leaderSpeed) / myGapControlGainSpace + myHeadwayTime * speed;
}


double
MSCFModel_Lin2016::insertionFollowSpeed(const MSVehicle* const v, double speed, double gap2pred, double predSpeed, double predMaxDecel, const MSVehicle* const /*pred*/) const {
//#ifdef DEBUG_LIN2016
//        std::cout << "MSCFModel_Lin2016::insertionFollowSpeed(), speed="<<speed<< std::endl;
//#endif
    // iterate to find a stationary value for
    //    speed = followSpeed(v, speed, gap2pred, predSpeed, predMaxDecel, nullptr)
    const int max_iter = 50;
    int n_iter = 0;
    const double tol = 0.1;
    const double damping = 0.1;

    double res = speed;
    while (n_iter < max_iter) {
        // proposed acceleration
        const double a = SPEED2ACCEL(followSpeed(v, res, gap2pred, predSpeed, predMaxDecel, nullptr) - res);
        res = res + damping * a;
//#ifdef DEBUG_LIN2016
//        std::cout << "   n_iter=" << n_iter << ", a=" << a << ", res=" << res << std::endl;
//#endif
        if (fabs(a) < tol) {
            break;
        } else {
            n_iter++;
        }
    }
    return res;
}


/// @todo update interactionGap logic
double
MSCFModel_Lin2016::interactionGap(const MSVehicle* const /* veh */, double /* vL */) const {
    /*maximum radar range is ACC is enabled*/
    return 250;
}

double MSCFModel_Lin2016::accelSpeedControl(double vErr) const {
    // Speed control law
    return mySpeedControlGain * vErr;
}

double MSCFModel_Lin2016::accelGapControl(const MSVehicle* const /* veh */, const double gap2pred, const double speed, const double predSpeed, double vErr) const {

#ifdef DEBUG_LIN2016
    if (DEBUG_COND) {
        std::cout << "        applying gapControl" << std::endl;
    }
#endif

// Gap control law
    double gclAccel = 0.0;
    double desSpacing = myHeadwayTime * speed;
    double spacingErr = gap2pred - desSpacing;
    double deltaVel = predSpeed - speed;


    if (fabs(spacingErr) < 0.2 && fabs(vErr) < 0.1) {
        // gap mode
        gclAccel = myGapControlGainSpeed * deltaVel + myGapControlGainSpace * spacingErr;
    } else if (spacingErr < 0)  {
        // collision avoidance mode
        gclAccel = myCollisionAvoidanceGainSpeed * deltaVel + myCollisionAvoidanceGainSpace * spacingErr;
    } else {
        // gap closing mode
        gclAccel = myGapClosingControlGainSpeed * deltaVel + myGapClosingControlGainSpace * spacingErr;
    }

    return gclAccel;
}

Position
MSCFModel_Lin2016::getRelativePosition(Position v1, double v1Heading, Position v2) const {
    // v1Heading: rad
    Position dummy(0.0, 0.0);
    Position result = (v2 - v1).rotateAround2D(-v1Heading, dummy);
    // std::cout<<v1<<","<<v1Heading<<" | "<<v2<<"\n\t"<<result<<"\n";
    return result;
}

const std::vector<const SUMOVehicle*>
MSCFModel_Lin2016::getInvolvedVehicles(const MSVehicle* const veh) const {

    std::vector<const SUMOVehicle*> vehs = {};
    double lookahead = myLookaheadDist;

    const MSLane* vehLane = veh->getLane();
    std::string vehEdgeID = Named::getIDSecure(veh->getLane()->getMyEdge());

    auto vehPosOnEdgeMap = MSNet::getInstance()->getEdgeControl().myVehPosOnEdgeMap;
    // std::shared_ptr<std::map<std::string, MSVehIDInstanceVector>> vehPosOnEdgeMap = 
    //     std::make_shared<std::map<std::string, MSVehIDInstanceVector>>(
    //         *MSNet::getInstance()->getEdgeControl().getVehPosOnEdgeMap());

    std::map<std::string, MSVehIDInstanceVector>::iterator itt = vehPosOnEdgeMap->begin();
    int vehCount = 0;
    while (itt != vehPosOnEdgeMap->end()) {
        std::string eid = itt->first;

        if (eid != "-18.0.00") {
            itt++;
            continue;
        }
        std::cout << "  " << eid << ": ";
        for (auto &vehPair: vehPosOnEdgeMap->find(eid)->second) {
            std::cout << vehPair.second->getID() << "(" << vehPair.first << ") ";
            vehCount += 1;
        }
        std::cout << std::endl;


        itt++;
    }

    const std::vector<MSLane*>& bestLaneConts = veh->getBestLanesContinuation();
    std::vector<MSLane*>::const_iterator it = bestLaneConts.begin();
    std::cout << "veh->getID() = " << veh->getID() << std::endl;
    while (lookahead > 0 && it != bestLaneConts.end()) {  // only iter lanes, no junctions
        bool isJunction = *it == nullptr;

        if (!isJunction) {
            std::string eid = (*it)->getEdge().getID();
            double laneLength = (*it)->getLength();
            bool foundSelf = false;
            double selfDist = 0;
            if (vehPosOnEdgeMap->find(eid) != vehPosOnEdgeMap->end()) {
                for (auto & distVeh: vehPosOnEdgeMap->at(eid)) {
                    if (distVeh.second->getID() == veh->getID()) {
                        foundSelf = true;
                        selfDist = distVeh.first;
                    }
                    else if (foundSelf){
                        double dist = distVeh.first - selfDist;
                        if (dist < lookahead) {
                            #ifdef DEBUG_GET_INVOLVED
                                if (DEBUG_COND){
                                    std::cout << "Push (" << eid << "): " << distVeh.second->getID() << std::endl;
                                }
                            #endif
                            vehs.push_back(distVeh.second);
                            lookahead -= dist;
                            selfDist = distVeh.first;
                        }
                        else {
                            lookahead = 0;
                            break;
                        }
                    }
                }
                std::cout << std::endl;
            }
            if (lookahead != 0) {
                if (selfDist <= laneLength) {
                    lookahead -= laneLength - selfDist;
                }
                else {
                    lookahead = 0;
                }
            }

            if (lookahead <= 0) {
                break;
            }

            // Append all vehicles in the following junction

            const MSLane* nextLane = (*it)->getCanonicalSuccessorLane();
            if (nextLane == nullptr) {
                break;
            }
            else {
                isJunction = true;
                std::string jid = nextLane->getEdge().getJunctionID();
                if (vehPosOnEdgeMap->find(jid) != vehPosOnEdgeMap->end()) {
                    for (auto & distVeh: vehPosOnEdgeMap->at(jid)) {
                        std::cout << "!!" << std::endl;
                        #ifdef DEBUG_GET_INVOLVED
                            if (DEBUG_COND){
                                std::cout << "Push: " << distVeh.second->getID() << std::endl;
                            }
                        #endif
                        vehs.push_back(distVeh.second);
                        std::cout << "Pushed: " << distVeh.second->getID() << std::endl;
                    }
                    // lookahead -= nextLane->getLength();
                    lookahead = 0;
                }
            }

        }
        else {  // Vehicle is now in a junction
            std::string jid = vehLane->getEdge().getJunctionID();
            if (vehPosOnEdgeMap->find(jid) != vehPosOnEdgeMap->end()) {
                for (auto & distVeh: vehPosOnEdgeMap->at(jid)) {
                    #ifdef DEBUG_GET_INVOLVED
                        if (DEBUG_COND){
                            std::cout << "Push: " << distVeh.second->getID() << std::endl;
                        }
                    #endif
                    vehs.push_back(distVeh.second);
                }
            }
            lookahead -= vehLane->getLength();
        }


        #ifdef DEBUG_GET_INVOLVED
            if (DEBUG_COND) {
                if (isJunction)  // When vehicle is at a junction
                    std::cout << vehLane->getEdge().getID() << " -> ";
                else {
                    std::cout << (*it)->getEdge().getID() << " -> ";  // Get Lane
                    const MSLane* nextLane = (*it)->getCanonicalSuccessorLane();
                    if (nextLane == nullptr)
                        std::cout << "NULL\n";
                    else {
                        if (it + 1 == bestLaneConts.end())  // Get Junction
                            std::cout << nextLane->getEdge().getJunctionID() << " -> NULL\n";
                        else
                            std::cout << nextLane->getEdge().getJunctionID() << " -> ";
                    }
                }
            }
        #endif
        ++it;
    }
    #ifdef DEBUG_GET_INVOLVED
        if (DEBUG_COND) {
            std::cout << "n = " << vehs.size() << ": ";
            for (auto & v: vehs) {
                std::cout << v->getID() << " -> ";
            }
            std::cout << std::endl << std::endl;
        }
    #endif
    return vehs;
}


double
MSCFModel_Lin2016::_v(const MSVehicle* const veh, const double gap2pred, const double speed,
                  const double predSpeed, const double desSpeed, const bool /* respectMinGap */) const {

    double accelACC = 0;
    double gapLimit_SC = GAP_THRESHOLD_SPEEDCTRL; // lower gap limit in meters to enable speed control law
    double gapLimit_GC = GAP_THRESHOLD_GAPCTRL; // upper gap limit in meters to enable gap control law

    const std::vector<const SUMOVehicle*> involved = getInvolvedVehicles(veh);
    Position currPos = veh->getPosition();
    double currHeading = veh->getAngle();
    if (involved.size()) {
        for (auto & inv: involved) {
            std::string vid = inv->getID();
            if (vid=="" || inv==0) {  // perhap it arrived at this point?
                std::cout << "pass" << std::endl;
                continue;
            }
            else {
                std::cout << vid << std::endl;
                Position invPosition = inv->getPosition();
                // if (invPosition != Position::INVALID) {  // Not in the net.
                    // double invHeading = inv->getAngle();
                    // double invHalfLength = inv->getLength() / 2.0;
                    // invPosition.setx(invPosition.x() - invHalfLength * cos(invHeading));
                    // invPosition.sety(invPosition.y() - invHalfLength * sin(invHeading));
                    // Position relativePosition = getRelativePosition(currPos, currHeading, invPosition);
                    // std::cout << inv->getID() << ": " << invPosition << " | " << relativePosition << std::endl;
                    // std::cout << inv->getID() << ": " << invPosition << " | " << invHeading << std::endl;
                // }
            }
        }
    }

#ifdef DEBUG_LIN2016
    if (DEBUG_COND) {
        std::cout << SIMTIME << " MSCFModel_Lin2016::_v() for veh '" << veh->getID() << "'\n"
                  << "        gap=" << gap2pred << " speed="  << speed << " predSpeed=" << predSpeed
                  << " desSpeed=" << desSpeed << std::endl;
    }
#endif


    /* Velocity error */
    double vErr = speed - desSpeed;
    int setControlMode = 0;
    Lin2016VehicleVariables* vars = (Lin2016VehicleVariables*) veh->getCarFollowVariables();
    if (vars->lastUpdateTime != MSNet::getInstance()->getCurrentTimeStep()) {
        vars->lastUpdateTime = MSNet::getInstance()->getCurrentTimeStep();
        setControlMode = 1;
    }
    if (gap2pred > gapLimit_SC) {

#ifdef DEBUG_LIN2016
        if (DEBUG_COND) {
            std::cout << "        applying speedControl" << std::endl;
        }
#endif
        // Find acceleration - Speed control law
        accelACC = accelSpeedControl(vErr);
        // Set cl to vehicle parameters
        if (setControlMode) {
            vars->ACC_ControlMode = 0;
        }
    } else if (gap2pred < gapLimit_GC) {
        // Find acceleration - Gap control law
        accelACC = accelGapControl(veh, gap2pred, speed, predSpeed, vErr);
        // Set cl to vehicle parameters
        if (setControlMode) {
            vars->ACC_ControlMode = 1;
        }
    } else {
        // Follow previous applied law
        int cm = vars->ACC_ControlMode;
        if (!cm) {

#ifdef DEBUG_LIN2016
            if (DEBUG_COND) {
                std::cout << "        applying speedControl" << std::endl;
            }
#endif
            accelACC = accelSpeedControl(vErr);
        } else {
            accelACC = accelGapControl(veh, gap2pred, speed, predSpeed, vErr);
        }

    }

    double newSpeed = speed + ACCEL2SPEED(accelACC);

#ifdef DEBUG_LIN2016
    if (DEBUG_COND) {
        std::cout << "        result: accel=" << accelACC << " newSpeed="  << newSpeed << std::endl;
    }
#endif

    return MAX2(0., newSpeed);
}


MSCFModel*
MSCFModel_Lin2016::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_Lin2016(vtype);
}
