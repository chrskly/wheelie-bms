/*
 * This file is part of the ev mustang bms project.
 *
 * Copyright (C) 2025 Christian Kelly <chrskly@chrskly.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BMS_SRC_INCLUDE_WEBSERVER_H_
#define BMS_SRC_INCLUDE_WEBSERVER_H_

/*
 * Bring up WiFi, mount LittleFS and start the HTTP server task.
 *
 * Safe to call unconditionally: it does nothing when WEB_INTERFACE_ENABLED is
 * 0, and it never fails fatally. If WiFi or the filesystem cannot be brought
 * up it says so on the console and returns, leaving the BMS running -- the web
 * interface is a convenience and must never be able to stop the battery
 * management doing its job.
 *
 * Call AFTER bms.start(), so the first snapshot is on its way by the time
 * anything can connect.
 */
void webserver_start();

#endif  // BMS_SRC_INCLUDE_WEBSERVER_H_
