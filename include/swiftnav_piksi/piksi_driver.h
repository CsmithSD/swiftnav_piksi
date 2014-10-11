/***************************************************************************//**
* \file piksi_driver.hpp
*
* \brief ROS Implementation of the C Driver (header)
* \author Scott K Logan
* \author Caleb Jamison
* \date February 23, 2014
*
* API for the ROS driver
*
* \section license License (BSD-3)
* Copyright (c) 2013, Scott K Logan\n
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* - Neither the name of Willow Garage, Inc. nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef _piksi_driver_hpp
#define _piksi_driver_hpp

#include "swiftnav_piksi/piksi.h"

#include <ros/ros.h>
#include <ros/rate.h>
#include <tf/tf.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>

#include <boost/thread.hpp>

namespace swiftnav_piksi
{
	class PIKSI
	{
	public:
		PIKSI( const ros::NodeHandle &_nh = ros::NodeHandle( ),
			const ros::NodeHandle &_nh_priv = ros::NodeHandle( "~" ),
			const std::string _port = "/dev/ttyACM0" );
		~PIKSI( );
		bool PIKSIOpen( );
		void PIKSIClose( );
	private:
		bool PIKSIOpenNoLock( );
		void PIKSICloseNoLock( );
		void spin( );
		void spinOnce( );
		/*!
		 * \brief Diagnostic update callback
		 *
		 * \author Scott K Logan
		 *
		 * Whenever the diagnostic_updater deems it necessary to update the values
		 * therein, this callback is called to fetch the values from the device.
		 *
		 * \param[out] stat Structure in which to store the values for
		 * diagnostic_updater to report
		 */
		void DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat );

		ros::NodeHandle nh;
		ros::NodeHandle nh_priv;
		std::string port;
		std::string frame_id;
		int piksid;
		boost::mutex cmd_lock;

		/*!
		 * \brief Diagnostic updater
		 */
		diagnostic_updater::Updater diag;
		/*!
		 * \brief Normal acceptable update rate minimum
		 */
		double min_update_rate;
		/*!
		 * \brief Normal acceptable update rate maximum
		 */
		double max_update_rate;
		/*!
		 * \brief Diagnostic rate for IMU publication
		 */
		diagnostic_updater::FrequencyStatus diag_pub_freq;

		ros::Publisher fix_pub;
		ros::Publisher status_pub;
		ros::Publisher time_pub;

		unsigned int io_failure_count;
		unsigned int open_failure_count;

		ros::Rate spin_rate;
		boost::thread spin_thread;
	};
}

#endif /* _piksi_driver_hpp */
