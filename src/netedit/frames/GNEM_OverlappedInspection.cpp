/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2022 German Aerospace Center (DLR) and others.
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
/// @file    GNEFrameModules.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Mar 2022
///
// Frame for overlapped elements
/****************************************************************************/
#include <config.h>

#include <netedit/GNEViewNet.h>
#include <netedit/frames/common/GNEInspectorFrame.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/windows/GUIAppEnum.h>

#include "GNEM_OverlappedInspection.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEM_OverlappedInspection) OverlappedInspectionMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_OVERLAPPED_NEXT,            GNEM_OverlappedInspection::onCmdNextElement),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_OVERLAPPED_PREVIOUS,        GNEM_OverlappedInspection::onCmdPreviousElement),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_OVERLAPPED_SHOWLIST,        GNEM_OverlappedInspection::onCmdShowList),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_OVERLAPPED_ITEMSELECTED,    GNEM_OverlappedInspection::onCmdListItemSelected),
    FXMAPFUNC(SEL_COMMAND,  MID_HELP,                           GNEM_OverlappedInspection::onCmdOverlappingHelp)
};


// Object implementation
FXIMPLEMENT(GNEM_OverlappedInspection,       FXGroupBoxModule,     OverlappedInspectionMap,        ARRAYNUMBER(OverlappedInspectionMap))


// ===========================================================================
// method definitions
// ===========================================================================

GNEM_OverlappedInspection::GNEM_OverlappedInspection(GNEFrame* frameParent) :
    FXGroupBoxModule(frameParent->getContentFrame(), "Overlapped elements"),
    myFrameParent(frameParent),
    myFilteredTag(SUMO_TAG_NOTHING),
    myItemIndex(0) {
    // build elements
    buildFXElements();
}


GNEM_OverlappedInspection::GNEM_OverlappedInspection(GNEFrame* frameParent, const SumoXMLTag filteredTag) :
    FXGroupBoxModule(frameParent->getContentFrame(), ("Overlapped " + toString(filteredTag) + "s").c_str()),
    myFrameParent(frameParent),
    myFilteredTag(filteredTag),
    myItemIndex(0) {
    // build elements
    buildFXElements();
}


GNEM_OverlappedInspection::~GNEM_OverlappedInspection() {}


void
GNEM_OverlappedInspection::showOverlappedInspection(const GNEViewNetHelper::ObjectsUnderCursor& objectsUnderCursor, const Position& clickedPosition) {
    // first clear myOverlappedACs
    myOverlappedACs.clear();
    // reserve
    myOverlappedACs.reserve(objectsUnderCursor.getClickedAttributeCarriers().size());
    // iterate over objects under cursor
    for (const auto& AC : objectsUnderCursor.getClickedAttributeCarriers()) {
        bool insert = true;
        // check supermode demand
        if (myFrameParent->getViewNet()->getEditModes().isCurrentSupermodeDemand() &&
                !AC->getTagProperty().isDemandElement()) {
            insert = false;
        }
        // check supermode data
        if (myFrameParent->getViewNet()->getEditModes().isCurrentSupermodeData() &&
                !AC->getTagProperty().isGenericData()) {
            insert = false;
        }
        // check filter
        if ((myFilteredTag != SUMO_TAG_NOTHING) && (AC->getTagProperty().getTag() != myFilteredTag)) {
            insert = false;
        }
        if (insert) {
            myOverlappedACs.push_back(AC);
        }
    }
    // continue depending of number of myOverlappedACs
    if (myOverlappedACs.size() > 1) {
        mySavedClickedPosition = clickedPosition;
        // by default we inspect first element
        myItemIndex = 0;
        // update text of current index button
        myCurrentIndexButton->setText(("1 / " + toString(myOverlappedACs.size())).c_str());
        // clear and fill list again
        myOverlappedElementList->clearItems();
        for (int i = 0; i < (int)myOverlappedACs.size(); i++) {
            myOverlappedElementList->insertItem(i, myOverlappedACs.at(i)->getID().c_str(), myOverlappedACs.at(i)->getIcon());
        }
        // set first element as selected element
        myOverlappedElementList->getItem(0)->setSelected(TRUE);
        // by default list hidden
        myOverlappedElementList->hide();
        // show GNEM_OverlappedInspection modul
        show();
    } else {
        // hide GNEM_OverlappedInspection modul
        hide();
    }
}


void
GNEM_OverlappedInspection::hideOverlappedInspection() {
    // hide GNEM_OverlappedInspection modul
    hide();
}


bool
GNEM_OverlappedInspection::overlappedInspectionShown() const {
    // show GNEM_OverlappedInspection modul
    return shown();
}


int
GNEM_OverlappedInspection::getNumberOfOverlappedACs() const {
    return (int)myOverlappedACs.size();
}


bool
GNEM_OverlappedInspection::checkSavedPosition(const Position& clickedPosition) const {
    return (mySavedClickedPosition.distanceSquaredTo2D(clickedPosition) < 0.25);
}


bool
GNEM_OverlappedInspection::nextElement(const Position& clickedPosition) {
    // first check if GNEM_OverlappedInspection is shown
    if (shown()) {
        // check if given position is near saved position
        if (checkSavedPosition(clickedPosition)) {
            // inspect next element
            onCmdNextElement(0, 0, 0);
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}


bool
GNEM_OverlappedInspection::previousElement(const Position& clickedPosition) {
    // first check if GNEM_OverlappedInspection is shown
    if (shown()) {
        // check if given position is near saved position
        if (checkSavedPosition(clickedPosition)) {
            // inspect previousElement
            onCmdPreviousElement(0, 0, 0);
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}


long
GNEM_OverlappedInspection::onCmdPreviousElement(FXObject*, FXSelector, void*) {
    // check if there is items
    if (myOverlappedElementList->getNumItems() > 0) {
        // unselect current list element
        myOverlappedElementList->getItem((int)myItemIndex)->setSelected(FALSE);
        // set index (it works as a ring)
        if (myItemIndex > 0) {
            myItemIndex--;
        } else {
            myItemIndex = (myOverlappedACs.size() - 1);
        }
        // selected current list element
        myOverlappedElementList->getItem((int)myItemIndex)->setSelected(TRUE);
        myOverlappedElementList->update();
        // update current index button
        myCurrentIndexButton->setText((toString(myItemIndex + 1) + " / " + toString(myOverlappedACs.size())).c_str());
        // inspect overlapped attribute carrier
        myFrameParent->selectedOverlappedElement(myOverlappedACs.at(myItemIndex));
        // show GNEM_OverlappedInspection again (because it's hidden in inspectSingleElement)
        show();
    }
    return 1;
}


long
GNEM_OverlappedInspection::onCmdNextElement(FXObject*, FXSelector, void*) {
    // check if there is items
    if (myOverlappedElementList->getNumItems() > 0) {
        // unselect current list element
        myOverlappedElementList->getItem((int)myItemIndex)->setSelected(FALSE);
        // set index (it works as a ring)
        myItemIndex = (myItemIndex + 1) % myOverlappedACs.size();
        // selected current list element
        myOverlappedElementList->getItem((int)myItemIndex)->setSelected(TRUE);
        myOverlappedElementList->update();
        // update current index button
        myCurrentIndexButton->setText((toString(myItemIndex + 1) + " / " + toString(myOverlappedACs.size())).c_str());
        // inspect overlapped attribute carrier
        myFrameParent->selectedOverlappedElement(myOverlappedACs.at(myItemIndex));
        // show GNEM_OverlappedInspection again (because it's hidden in inspectSingleElement)
        show();
    }
    return 1;
}


long
GNEM_OverlappedInspection::onCmdShowList(FXObject*, FXSelector, void*) {
    // show or hidde element list
    if (myOverlappedElementList->shown()) {
        myOverlappedElementList->hide();
    } else {
        myOverlappedElementList->show();
    }
    if (myOverlappedElementList->getNumItems() <= 10) {
        myOverlappedElementList->setHeight(23 * myOverlappedElementList->getNumItems());
    } else {
        myOverlappedElementList->setHeight(230);
    }
    myOverlappedElementList->recalc();
    // recalc and update frame
    recalc();
    return 1;
}

long
GNEM_OverlappedInspection::onCmdListItemSelected(FXObject*, FXSelector, void*) {
    for (int i = 0; i < myOverlappedElementList->getNumItems(); i++) {
        if (myOverlappedElementList->getItem(i)->isSelected()) {
            myItemIndex = i;
            // update current index button
            myCurrentIndexButton->setText((toString(myItemIndex + 1) + " / " + toString(myOverlappedACs.size())).c_str());
            // inspect overlapped attribute carrier
            myFrameParent->selectedOverlappedElement(myOverlappedACs.at(myItemIndex));
            // show GNEM_OverlappedInspection again (because it's hidden in inspectSingleElement)
            show();
            return 1;
        }
    }
    return 0;
}


long
GNEM_OverlappedInspection::onCmdOverlappingHelp(FXObject*, FXSelector, void*) {
    FXDialogBox* helpDialog = new FXDialogBox(getCollapsableFrame(), "GEO attributes Help", GUIDesignDialogBox);
    std::ostringstream help;
    help
            << " - Click in the same position\n"
            << "   for inspect next element\n"
            << " - Shift + Click in the same\n"
            << "   position for inspect\n"
            << "   previous element";
    new FXLabel(helpDialog, help.str().c_str(), nullptr, GUIDesignLabelFrameInformation);
    // "OK"
    new FXButton(helpDialog, "OK\t\tclose", GUIIconSubSys::getIcon(GUIIcon::ACCEPT), helpDialog, FXDialogBox::ID_ACCEPT, GUIDesignButtonOK);
    helpDialog->create();
    helpDialog->show();
    return 1;
}


GNEM_OverlappedInspection::GNEM_OverlappedInspection() :
    myFrameParent(nullptr),
    myPreviousElement(nullptr),
    myCurrentIndexButton(nullptr),
    myNextElement(nullptr),
    myOverlappedElementList(nullptr),
    myHelpButton(nullptr),
    myFilteredTag(SUMO_TAG_NOTHING),
    myItemIndex(0) {
}


void
GNEM_OverlappedInspection::buildFXElements() {
    FXHorizontalFrame* frameButtons = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    // Create previous Item Button
    myPreviousElement = new FXButton(frameButtons, "", GUIIconSubSys::getIcon(GUIIcon::BIGARROWLEFT), this, MID_GNE_OVERLAPPED_PREVIOUS, GUIDesignButtonIconRectangular);
    // create current index button
    myCurrentIndexButton = new FXButton(frameButtons, "", nullptr, this, MID_GNE_OVERLAPPED_SHOWLIST, GUIDesignButton);
    // Create next Item Button
    myNextElement = new FXButton(frameButtons, "", GUIIconSubSys::getIcon(GUIIcon::BIGARROWRIGHT), this, MID_GNE_OVERLAPPED_NEXT, GUIDesignButtonIconRectangular);
    // Create list of overlapped elements (by default hidden)
    myOverlappedElementList = new FXList(getCollapsableFrame(), this, MID_GNE_OVERLAPPED_ITEMSELECTED, GUIDesignListFixedHeight);
    // by default list of overlapped elements is hidden)
    myOverlappedElementList->hide();
    // Create help button
    myHelpButton = new FXButton(getCollapsableFrame(), "Help", nullptr, this, MID_HELP, GUIDesignButtonRectangular);
}

/****************************************************************************/