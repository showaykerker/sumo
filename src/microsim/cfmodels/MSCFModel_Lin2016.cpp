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
#include <microsim/MSVehicleControl.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOTime.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <math.h>

// ===========================================================================
// debug flags
// ===========================================================================
//#define DEBUG_LIN2016
// #define DEBUG_COND (true)
// #define DEBUG_GET_INVOLVED
// #define DEBUG_COND (veh->isSelected())
// #define DEBUG_RELATIVE_POS

typedef std::vector<MSEdge*> MSEdgeVector;
typedef std::vector<std::pair<double, std::string>> MSVehIDInstanceVector;

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

#define DEFAULT_LOOKAHEAD 25.0
#define DEFAULT_SYMIN 0.1  // s^y_min in IV.A

#define DEFAULT_FREE_ACC_EXPONENT 4  // delta in IV.B.(2)
#define DEFAULT_MAX_ACCELERATION 6.0  // a^x_max in IV.B.(2)
#define DEFAULT_COMFORTABLE_DECELERATION 4.0  // b^x_com in IV.B.(4)
#define DEFAULT_H 1.2  // h in IV.B.(4)
#define DEFAULT_DESIRED_TIME_HEADAWAY 4.0  // t^x cap in IV.B.(5)
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
    myCalculatedNewSpeed({}),
    myLookaheadDist(vtype->getParameter().getCFParam(SUMO_ATTR_CF_L16_LOOKAHEAD, DEFAULT_LOOKAHEAD)),
    myFreeAccExponent(vtype->getParameter().getCFParam(SUMO_ATTR_CF_L16_FREE_ACC_EXPONENT, DEFAULT_FREE_ACC_EXPONENT)),
    myMaxAcceleration(vtype->getParameter().getCFParam(SUMO_ATTR_CF_L16_MAX_ACCELERATION, DEFAULT_MAX_ACCELERATION)),
    myComfortableDeceleration(vtype->getParameter().getCFParam(SUMO_ATTR_CF_L16_COMFORTABLE_DECELERATION, DEFAULT_COMFORTABLE_DECELERATION)),
    myHConstant(vtype->getParameter().getCFParam(SUMO_ATTR_CF_L16_H, DEFAULT_H)),

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

MSCFModel_Lin2016::InvolvedVehicleInfo
MSCFModel_Lin2016::CalculateInvolvedVehicleInfo(const MSVehicle* const veh, std::string involvedVehicleID) const {
    SUMOVehicle* inv = MSNet::getInstance()->getVehicleControl().getVehicle(involvedVehicleID);
    InvolvedVehicleInfo info;

    info.valid = false;
    double phiJ = -1;
    double actualGap = -1;

    if (inv) {  // Vehicle that is near the arrival will cause error
        info.valid = true;
        Position invPosition = inv->getPosition();
        info.speed = inv->getSpeed();
        if (invPosition != Position::INVALID) {  // Not in the net.

            invPosition.setx(invPosition.x() - inv->getLength() / 2.0 * cos(inv->getAngle()));
            invPosition.sety(invPosition.y() - inv->getLength() / 2.0 * sin(inv->getAngle()));
            Position relativePosition = getRelativePosition(veh->getPosition(), veh->getAngle(), invPosition);
            double relativeHeading = inv->getAngle() - veh->getAngle();
            while (relativeHeading < -PI) relativeHeading += PI;
            while (relativeHeading > PI) relativeHeading -= PI;

            #ifdef DEBUG_RELATIVE_POS
                std::cout << "\t" << inv->getID() << ": " << invPosition << " | ";
                std::cout << relativePosition << "," << relativeHeading << std::endl;
            #endif

            if (relativePosition.x() > 0) {
                double wj = inv->getVehicleType().getWidth();
                double diagnalLength = pow(
                    pow(wj, 2) + pow(inv->getLength(), 2), 0.5);
                double alpha = atan2(inv->getLength(), wj);
                double shaded = diagnalLength * sin(alpha + abs(relativeHeading));
                double sYMax = veh->getWidth() / 2.0 + shaded / 2.0 + DEFAULT_SYMIN;
                // double phiJTilde = 1 - (shaded / 2.0 + veh->getWidth() / 2.0) / sYMax;

                double delta = relativePosition.x();
                double wJCap = shaded * 2.0;
                double w = veh->getWidth();
                double x0 = relativePosition.x() - shaded / 2.0;
                double x1 = relativePosition.x() + shaded / 2.0;

                actualGap = relativePosition.x();
                if (abs(delta) >= sYMax) {  // Not a potential leader
                    phiJ = 0;
                }
                else if (wJCap >= w){
                    if (abs(delta) <= (wJCap - w) / 2.0) {
                        phiJ = 1;
                    }
                    else if ((wJCap - w) / 2.0 < abs(delta) && abs(delta) < sYMax) {
                        phiJ = 1 - (w/2 - std::min(abs(x0), abs(x1))) / sYMax;
                    }
                }
                else if (wJCap < w) {
                    if (abs(delta) <= (w - wJCap) / 2.0) {
                        phiJ = 1 - ( ((w-wJCap)/2.0) * ( 2 * abs(delta) / (w - wJCap) ) ) / sYMax;
                    }
                    else if ((w - wJCap) / 2.0 < abs(delta) && abs(delta) < sYMax) {
                        if (delta > 0) {
                            phiJ = 1 - (x0 + w / 2.0) / sYMax;
                        }
                        else if (delta < 0) {
                            phiJ = 1 - (w/2.0 - x1) / sYMax;
                        }
                    }
                }

            }
            else {
                phiJ = 0;
                actualGap = 1000;
            }

        }
    }
    info.lateralOverlappingRatio = phiJ;
    info.actualGap = actualGap;
    return info;
}

const std::vector<std::string>
MSCFModel_Lin2016::getInvolvedVehicles(const MSVehicle* const veh) const {

    std::vector<std::string> vehs = {};
    double lookahead = myLookaheadDist;

    const MSLane* vehLane = veh->getLane();
    std::string vehEdgeID = Named::getIDSecure(veh->getLane()->getMyEdge());

    auto vehPosOnEdgeMap = MSNet::getInstance()->getEdgeControl().getVehPosOnEdgeMap();

    const std::vector<MSLane*>& bestLaneConts = veh->getBestLanesContinuation();
    std::vector<MSLane*>::const_iterator it = bestLaneConts.begin();
    while (lookahead > 0 && it != bestLaneConts.end()) {  // only iter lanes, no junctions
        bool isJunction = *it == nullptr;

        if (!isJunction) {
            std::string eid = (*it)->getEdge().getID();
            double laneLength = (*it)->getLength();
            bool foundSelf = false;
            double selfDist = 0;
            double currentDist = 0;
            if (vehPosOnEdgeMap->find(eid) != vehPosOnEdgeMap->end()) {
                for (auto & distVeh: vehPosOnEdgeMap->at(eid)) {
                    if (distVeh.second == veh->getID()) {
                        foundSelf = true;
                        selfDist = distVeh.first;
                        currentDist = distVeh.first;
                    }
                    else if (foundSelf){
                        double dist = distVeh.first - currentDist;
                        if (selfDist != distVeh.first) {
                            if (dist < lookahead) {
                                #ifdef DEBUG_GET_INVOLVED
                                    if (DEBUG_COND){
                                        std::cout << "Push (" << eid << "): " << distVeh.second << std::endl;
                                    }
                                #endif
                                vehs.push_back(distVeh.second);
                                lookahead -= dist;
                                currentDist = distVeh.first;
                            }
                            else {
                                lookahead = 0;
                                break;
                            }
                        }
                    }
                }
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
                        #ifdef DEBUG_GET_INVOLVED
                            if (DEBUG_COND){
                                std::cout << "Push: " << distVeh.second << std::endl;
                            }
                        #endif
                        vehs.push_back(distVeh.second);
                    }
                    lookahead -= nextLane->getLength();
                    // lookahead = 0;
                }
            }

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
                std::cout << v << " -> ";
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
    double newSpeed;

    const SUMOTime currTime = MSNet::getInstance()->getCurrentTimeStep();

    if (myCalculatedNewSpeed.find(veh->getID()) != myCalculatedNewSpeed.end()) {
        std::pair<SUMOTime, double> timeSpeedPair = myCalculatedNewSpeed.at(veh->getID());
        if (timeSpeedPair.first == currTime)
            return timeSpeedPair.second;
    }

    if (veh->getEdge()->isJunctionConst()) {
        const std::vector<std::string> involved = getInvolvedVehicles(veh);
        if (involved.size()) {
            double vX = veh->getSpeed();
            double tXCap = DEFAULT_DESIRED_TIME_HEADAWAY;
            double rootAXMaxBXCom = pow(myMaxAcceleration * myComfortableDeceleration, 0.5);
            double maxDeceleration = -1;
            for (auto & vid: involved) {
                InvolvedVehicleInfo inv = CalculateInvolvedVehicleInfo(veh, vid);
                if (!inv.valid) continue;
                double phi = inv.lateralOverlappingRatio;
                double vXJ = inv.speed;
                double sXJ = inv.actualGap;
                double sXJCap = phi + vX * tXCap + (vX * (vX - vXJ))/(2 * rootAXMaxBXCom); // desiredGap

                // activation govening control
                int activate = (vX - vXJ) > 0 || (myHConstant * (sXJCap - sXJ)) > 0;
                double bXJ = myComfortableDeceleration * pow(sXJCap/sXJ, 2) * activate;
                if (bXJ > maxDeceleration) {
                    maxDeceleration = bXJ;
                }
            }
            double aXFree = myMaxAcceleration * (
                1 - pow( vX / veh->getMaxSpeed(), myFreeAccExponent));  // longitudinalFreeAcceleration
            accelACC = aXFree - maxDeceleration;
        }
        newSpeed = speed + ACCEL2SPEED(accelACC);
    }
    else {
        double gapLimit_SC = GAP_THRESHOLD_SPEEDCTRL; // lower gap limit in meters to enable speed control law
        double gapLimit_GC = GAP_THRESHOLD_GAPCTRL; // upper gap limit in meters to enable gap control law
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

            newSpeed = speed + ACCEL2SPEED(accelACC);

        #ifdef DEBUG_LIN2016
            if (DEBUG_COND) {
                std::cout << "        result: accel=" << accelACC << " newSpeed="  << newSpeed << std::endl;
            }
        #endif
    }

    // myCalculatedNewSpeed.insert(
    //     std::map<std::string, std::pair<SUMOTime, double>>(
    //         veh->getID(), std::pair<SUMOTime, double>(currTime, MAX2(0., newSpeed))
    //     )
    // );

    return MAX2(0., newSpeed);
}


MSCFModel*
MSCFModel_Lin2016::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_Lin2016(vtype);
}
