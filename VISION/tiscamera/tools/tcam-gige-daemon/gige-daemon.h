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

#ifndef TCAM_GIGE_DAEMON_H
#define TCAM_GIGE_DAEMON_H

#include "../../src/base_types.h"

namespace tcam::tools::gige_daemon
{

static const size_t TCAM_DEVICE_LIST_MAX = 50;

struct tcam_gige_device_list
{
    unsigned int device_count;

    tcam::tcam_device_info devices[TCAM_DEVICE_LIST_MAX];
};

constexpr const char* LOCK_FILE = "/var/lock/tcam-gige-daemon.lock";

} // namespace tcam::tools::gige_daemon

#endif /* TCAM_GIGE_DAEMON_H */
