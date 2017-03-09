/*
 * progressBar.cpp
 *
 *  Created on: Aug 6, 2014
 *      Author: weidenka
 */

#include <chrono>
#include "Core/ProgressBar.hpp"


namespace ComPWA {
progressBar::progressBar(int size, int update) : hasStarted(0), numEvents(size), updateInterval(update),lastUpdate(0) {
	if(update==0) updateInterval = 1;

}
void progressBar::nextEvent(){
	if(!hasStarted){
		lastUpdate = 0;
		currentEvent = 0;
		time(&startTime);
		hasStarted=1;
		update();
		fflush(stdout);
	}
	currentEvent++;
	if( (int)((timePassed()-lastUpdate)) > updateInterval ) update();
	if( currentEvent == numEvents ) {
		update();
		std::cout<<std::endl;
	}

}
void progressBar::update(){
//	std::cout<<"update()"<<std::endl;
//	std::cout<<timePassed()<<" "<<lastUpdate<<" "<<(int)((timePassed()-lastUpdate))<<std::endl;
	currentPercent = (double)currentEvent/numEvents * 100;
	int nStars = ((int) (currentPercent/10+0.5));
	char buf[10];
	int i=0;
	for( ; i<nStars; i++) buf[i]='*';
	for( ; i<10; i++) buf[i]='-';
//	for( ; i<15; i++) buf[i]=' ';
	time_t estEndTime = endTime();
	char timebuf[10];
	strftime(timebuf,10,"%I:%M%p",localtime(&estEndTime));
	//the escape sequence \e[?25l switches off the courser and \e[?25h switch it on again
	printf("\e[?25l \r %4.2fmin [ %.*s %4.2f%% ] %4.2fmin , end time: %7s               \e[?25h",
			timePassed()/60,10,buf, currentPercent, timeRemaining()/60,timebuf);
	fflush(stdout);

	lastUpdate = timePassed();
	return;
}

double progressBar::timeRemaining(){
	if(currentPercent==0) return 0.0;
	return timePassed()*(1/currentPercent*100-1);
}
double progressBar::timePassed(){
	time_t currentTime;
	time(&currentTime);
	return difftime(currentTime,startTime);
}

time_t progressBar::endTime(){
	time_t currentTime;
	time(&currentTime);
	time_t estEndTime = currentTime+timeRemaining();
	return estEndTime;
}

}
