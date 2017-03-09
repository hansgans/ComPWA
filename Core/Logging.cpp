/*
 * Logging.cpp
 *
 *  Created on: Mar 19, 2014
 *      Author: weidenka
 */

#include <boost/log/expressions.hpp>
#include <boost/log/expressions/formatters/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "Core/Logging.hpp"

namespace ComPWA {
void Logging::init(std::string out,boost::log::trivial::severity_level minLevel){
   boost::log::add_common_attributes();
   boost::log::add_console_log(std::cout,
		 boost::log::keywords::format =
		 (
		  boost::log::expressions::stream
		  << boost::log::expressions::format_date_time< boost::posix_time::ptime >("TimeStamp", "%H:%M:%S")
		  << " [" << std::setw(7) << boost::log::trivial::severity<< "] : "
		  << boost::log::expressions::smessage
		 )
		 );
   if(out == ""){
	  BOOST_LOG_TRIVIAL(info)<<"Logging: logging to filename disables. Console severity level: "<<minLevel;
   } else {
	  boost::log::add_file_log( boost::log::keywords::file_name=out,
			//			keywords::format="(%LineID%) [%TimeStamp%][%Severity%]: %Message%"
			boost::log::keywords::format =
			(
			 boost::log::expressions::stream
			 << boost::log::expressions::format_date_time< boost::posix_time::ptime >("TimeStamp", "%Y-%m-%d %H:%M:%S")
			 << " [" << boost::log::trivial::severity<< "] : "
			 << boost::log::expressions::smessage
			)
			);
	  boost::log::core::get()->set_filter(boost::log::trivial::severity >= minLevel);
	  BOOST_LOG_TRIVIAL(info)<<"Logging: using output filename: "<<out<<", Severity level: "<<minLevel;
   }

   //Print local time and date at the beginning
   boost::posix_time::ptime todayUtc(
		 boost::gregorian::day_clock::universal_day(), 
		 boost::posix_time::second_clock::local_time().time_of_day()
		 );
   BOOST_LOG_TRIVIAL(info) << "Current date and time: "<<boost::posix_time::to_simple_string(todayUtc);
}
void Logging::setLogLevel(boost::log::trivial::severity_level minLevel){
   boost::log::core::get()->set_filter(boost::log::trivial::severity >= minLevel);
   BOOST_LOG_TRIVIAL(info)<<"New severity level: "<<minLevel;
}
}
