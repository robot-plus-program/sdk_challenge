/*
 * Copyright 2016 The Imaging Source Europe GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "FirmwareUpgrade.h"
#include "GigE3UploadItem.h"

#include <pugi.h>
#include <string>
#include <vector>

class TiXmlElement;

namespace FirmwareUpdate
{

namespace GigE3
{

class IDevicePort
{
public:
    virtual std::string name() = 0;


public:
    virtual Status Configure(const std::string& name, const pugi::xml_node& portConfigElem) = 0;

    // Check whether all the items are consistent with the requirements of this device port
    virtual Status CheckItems(const std::vector<UploadItem>& items) = 0;

    // Upload all items through the device port
    virtual Status UploadItems(IFirmwareWriter& dev,
                               const std::vector<UploadItem>& items,
                               tReportProgressFunc progressFunc) = 0;
}; /* class IDevicePort */

} /* namespace GigE3 */

} /* namespace FirmwareUpdate */
